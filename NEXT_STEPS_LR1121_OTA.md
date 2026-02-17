# LR1121 OTA Firmware Update Implementation Plan

## Current Status ✅
- WiFi AP working (`ELRS_TEST_AP` / `elrs1234` on `192.168.10.1`)
- Web UI serving correctly (index.html, CSS, JS)
- `/config` GET/POST working - saves binding phrase, UID, etc to NVM3
- `/reboot` working - triggers NVIC_SystemReset()
- Config persistence verified across power cycles

## Goal
Implement LR1121 Semtech LoRa radio firmware flashing via the ELRS Web UI, matching the ESP32 ELRS 4.0 implementation.

---

## Phase 1: LR1121 Firmware Version API

### Endpoint: `GET /lr1121.json`

Returns JSON with current LR1121 firmware info:
```json
{
  "manual": false,
  "radio1": {
    "hardware": 34,
    "type": 3,
    "firmware": 258
  }
}
```

### Implementation in `lr1121_driver.c`:

```c
// Command: LR11XX_SYSTEM_GET_VERSION (0x0101)
// Response: [status][hw][type][fw_msb][fw_lsb]
typedef struct {
    uint8_t hardware;  // Hardware version (e.g., 0x22 for LR1121)
    uint8_t type;      // 0x03 = transceiver firmware, 0xDF = bootloader
    uint16_t version;  // Firmware version (e.g., 0x0102 = v1.2)
} lr1121_firmware_version_t;

lr1121_firmware_version_t lr1121_get_firmware_version(void);
```

### HTTP Handler in `wifi_http_test.c`:
```c
static sl_status_t handle_lr1121_status(sl_http_server_t *handle, sl_http_server_request_t *req);
// Register: { .uri = "/lr1121.json", .handler = handle_lr1121_status }
```

---

## Phase 2: LR1121 Bootloader Commands

### Key Commands (from LR1121_Regs.h):

| Command | Opcode | Description |
|---------|--------|-------------|
| `LR11XX_SYSTEM_REBOOT_OC` | 0x0118 | Reboot (mode=3 for bootloader) |
| `LR11XX_BL_GET_VERSION_OC` | 0x0101 | Get bootloader version |
| `LR11XX_BL_ERASE_FLASH_OC` | 0x8000 | Erase flash (takes ~2-3 seconds) |
| `LR11XX_BL_WRITE_FLASH_ENCRYPTED_OC` | 0x8003 | Write encrypted firmware chunk |
| `LR11XX_BL_REBOOT_OC` | 0x8005 | Reboot from bootloader to app |

### Functions to implement in `lr1121_driver.c`:

```c
/**
 * Begin firmware update - puts LR1121 in bootloader mode and erases flash
 * @param expected_size Total firmware file size in bytes
 * @return 0 on success, -1 if failed to enter bootloader mode
 */
int lr1121_begin_update(uint32_t expected_size);

/**
 * Write firmware data chunk (internally buffers to 256-byte blocks)
 * @param data Pointer to firmware data
 * @param size Number of bytes
 * @return 0 on success
 */
int lr1121_write_update_bytes(const uint8_t *data, uint32_t size);

/**
 * Complete firmware update - writes remaining data and reboots
 * @return 0 on success, -1 if not enough data, -2 if still in bootloader
 */
int lr1121_end_update(void);
```

### Firmware Update Flow:
1. **BeginUpdate**:
   - Reboot to bootloader: `SYSTEM_REBOOT_OC` with mode=3
   - Wait for BUSY to go low
   - Verify bootloader mode: `BL_GET_VERSION_OC` should return type=0xDF
   - Erase flash: `BL_ERASE_FLASH_OC` (wait ~3 seconds)

2. **WriteUpdateBytes**:
   - Buffer incoming data
   - When 256 bytes accumulated, send with `BL_WRITE_FLASH_ENCRYPTED_OC`
   - Command format: [opcode 2B][address 4B][data 256B]

3. **EndUpdate**:
   - Flush remaining buffered data
   - Reboot: `BL_REBOOT_OC` with param=0
   - Verify NOT in bootloader mode (type != 0xDF)

---

## Phase 3: HTTP Firmware Upload Endpoint

### Endpoint: `POST /lr1121`

**Headers expected:**
- `X-FileSize`: Total file size in bytes
- `X-Radio`: Which radio to update (1 or 2, default 1)
- `Content-Type`: `multipart/form-data`

**Response:**
```json
{"status": "ok", "msg": "Update complete. Refresh page to see new version."}
```
or
```json
{"status": "error", "msg": "Not enough data uploaded!"}
```

### Implementation:

```c
// State for ongoing firmware upload
static struct {
    uint32_t expected_size;
    uint32_t received_size;
    uint8_t buffer[256];
    uint32_t buffer_len;
    bool in_progress;
} lr1121_upload_state;

static sl_status_t handle_lr1121_upload(sl_http_server_t *handle, sl_http_server_request_t *req);
```

**Challenges:**
- SiWx917 HTTP server may not support streaming multipart uploads natively
- May need to receive entire file first, then flash
- Alternative: Accept raw binary POST body (simpler)

---

## Phase 4: Web UI Integration

The ELRS 4.0 Web UI already has `lr1121-updater.js` component. Need to:

1. Rebuild web assets with LR1121 updater enabled
2. Update `elrs_web_content.h` with new gzip data
3. Add navigation link to LR1121 updater page

### Files in ELRS 4.0:
- `src/html/src/pages/lr1121-updater.js` - The UI component
- `src/html/headers/web-lr1121-rx.h` - Pre-built header for LR1121 RX

---

## Phase 5: Testing

### Test Plan:
1. **Version Read Test**: Verify `/lr1121.json` returns valid firmware version
2. **Bootloader Entry Test**: Verify can enter/exit bootloader mode
3. **Small File Test**: Upload small test binary, verify write
4. **Full Firmware Test**: Upload actual LR1121 firmware (~250KB), verify complete
5. **Radio Function Test**: After update, verify LoRa TX/RX still works

### Test Files:
- LR1121 firmware binary from Semtech or ELRS releases
- Located in ELRS at: `src/lib/LR1121Driver/lr1121_transceiver_F30104.h` (embedded)

---

## File Changes Required

### `lr1121_driver.h` - Add:
```c
typedef struct {
    uint8_t hardware;
    uint8_t type;
    uint16_t version;
} lr1121_firmware_version_t;

lr1121_firmware_version_t lr1121_get_firmware_version(void);
int lr1121_begin_update(uint32_t expected_size);
int lr1121_write_update_bytes(const uint8_t *data, uint32_t size);
int lr1121_end_update(void);
```

### `lr1121_driver.c` - Add:
- Implement the 4 functions above
- Use existing SPI infrastructure

### `wifi_http_test.c` - Add:
```c
static sl_status_t handle_lr1121_status(sl_http_server_t *handle, sl_http_server_request_t *req);
static sl_status_t handle_lr1121_upload(sl_http_server_t *handle, sl_http_server_request_t *req);

// In handler table:
{ .uri = "/lr1121.json", .handler = handle_lr1121_status },
{ .uri = "/lr1121",      .handler = handle_lr1121_upload },
```

---

## Timeline Estimate

| Phase | Effort | Description |
|-------|--------|-------------|
| Phase 1 | 1-2 hours | Version API - simple SPI command |
| Phase 2 | 3-4 hours | Bootloader commands - careful timing |
| Phase 3 | 2-3 hours | HTTP upload handler |
| Phase 4 | 1-2 hours | Web UI integration |
| Phase 5 | 2-3 hours | Testing & debugging |

**Total: ~10-14 hours**

---

## References

- **ELRS 4.0 Source**: `C:\Users\mjeuw\ELRS-Port\ELRS_4.0_Clean\src\lib\WIFI\lr1121.cpp`
- **LR1121 Driver**: `C:\Users\mjeuw\ELRS-Port\ELRS_4.0_Clean\src\lib\LR1121Driver\LR1121.cpp`
- **LR1121 Registers**: `C:\Users\mjeuw\ELRS-Port\ELRS_4.0_Clean\src\lib\LR1121Driver\LR1121_Regs.h`
- **LR1121 Datasheet**: `61252685.LR1121_V2_1_data_sheet.pdf` (in document search)
- **Current Project**: `C:\Users\mjeuw\SimplicityStudio\TEST\wifi_gspi_merged\`
