#include "frame_buffer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_heap_caps.h"
#include "esp_log.h"

/* Ring storage (FB_CAPACITY * 24 B ≈ 48 KB). Allocated in PSRAM in fb_init() to keep this large
   buffer out of the scarce internal DRAM that BLE/FATFS need; falls back to internal DRAM if PSRAM
   is unavailable. Accessed only from tasks (never an ISR), so it's safe against the flash/PSRAM
   cache window (flash ops stall the other core rather than letting it fault on PSRAM). */
static CanFrame_t *s_ring = NULL;

/* Absolute (never-wrapping) sequence counters. count = head - tail, always <= FB_CAPACITY. */
static uint64_t s_head = 0;   /* index of the next slot to write (== total ever pushed) */
static uint64_t s_tail = 0;   /* index of the oldest valid frame */
static uint32_t s_dropped = 0;

static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;

void fb_init(void)
{
  if (s_ring == NULL)
  {
    /* Prefer PSRAM for the 48 KB ring; fall back to internal DRAM if PSRAM isn't present. */
    s_ring = (CanFrame_t *)heap_caps_malloc(sizeof(CanFrame_t) * FB_CAPACITY, MALLOC_CAP_SPIRAM);
    if (s_ring == NULL)
    {
      s_ring = (CanFrame_t *)heap_caps_malloc(sizeof(CanFrame_t) * FB_CAPACITY, MALLOC_CAP_8BIT);
      ESP_LOGW("FB", "ring in internal DRAM (%u B) — PSRAM unavailable", (unsigned)(sizeof(CanFrame_t) * FB_CAPACITY));
    }
    else
    {
      ESP_LOGI("FB", "ring in PSRAM (%u B)", (unsigned)(sizeof(CanFrame_t) * FB_CAPACITY));
    }
  }

  portENTER_CRITICAL(&s_mux);
  s_head = 0;
  s_tail = 0;
  s_dropped = 0;
  portEXIT_CRITICAL(&s_mux);
}

void fb_push(const CanFrame_t *frame)
{
  if (frame == NULL || s_ring == NULL)
  {
    return;
  }

  portENTER_CRITICAL(&s_mux);

  if ((s_head - s_tail) >= FB_CAPACITY)
  {
    /* Full: drop the oldest to make room for the newest. */
    s_tail++;
    s_dropped++;
  }

  s_ring[s_head % FB_CAPACITY] = *frame;
  s_head++;

  portEXIT_CRITICAL(&s_mux);
}

size_t fb_count(void)
{
  size_t n;
  portENTER_CRITICAL(&s_mux);
  n = (size_t)(s_head - s_tail);
  portEXIT_CRITICAL(&s_mux);
  return n;
}

size_t fb_peek(CanFrame_t *out, size_t max, uint64_t *out_first_seq)
{
  if (out == NULL || max == 0 || s_ring == NULL)
  {
    return 0;
  }

  size_t copied = 0;

  portENTER_CRITICAL(&s_mux);

  uint64_t seq = s_tail;
  if (out_first_seq != NULL)
  {
    *out_first_seq = seq;
  }

  uint64_t available = s_head - s_tail;
  size_t n = (available < (uint64_t)max) ? (size_t)available : max;

  for (size_t i = 0; i < n; i++)
  {
    out[i] = s_ring[(seq + i) % FB_CAPACITY];
  }
  copied = n;

  portEXIT_CRITICAL(&s_mux);

  return copied;
}

void fb_commit(uint64_t upto_seq)
{
  portENTER_CRITICAL(&s_mux);
  /* Only advance forward, and never past what has actually been produced. Overflow drops may
     have already moved the tail beyond upto_seq — in that case this is a no-op. */
  if (upto_seq > s_tail)
  {
    s_tail = (upto_seq < s_head) ? upto_seq : s_head;
  }
  portEXIT_CRITICAL(&s_mux);
}

uint32_t fb_dropped(void)
{
  uint32_t d;
  portENTER_CRITICAL(&s_mux);
  d = s_dropped;
  portEXIT_CRITICAL(&s_mux);
  return d;
}
