#!/usr/bin/env python
import re

# --- Configuration ---
ETU = 128
TOLERANCE = 0.25 * ETU

# ==============================================================================
# DECODER (UNCHANGED)
# ==============================================================================

def decode_pulses_to_bits(pulses):
    """
    Decodes a pulse train using a state machine. Returns the decoded bits
    and the state of the last bit that caused a pulse. This function is
    already capable of handling any length of bitstream.
    """
    if not pulses:
        return "", '?'
    bits = "0"
    for pulse in pulses:
        if bits[-1] == '0':
            if abs(pulse - ETU) < TOLERANCE:
                bits += '0';
            elif abs(pulse - (1.5 * ETU)) < TOLERANCE:
                bits += '1';
            else:
                return bits + f"[err:pulse={pulse},last_bit=0]", bits[-1]
        elif bits[-1] == '1':
            if abs(pulse - ETU) < TOLERANCE:
                bits += '1';
            elif abs(pulse - (1.5 * ETU)) < TOLERANCE:
                bits += '00';
            elif abs(pulse - (2 * ETU)) < TOLERANCE:
                bits += '01';
            else:
                return bits + f"[err:pulse={pulse},last_bit=1]", bits[-1]
    return bits

# ==============================================================================
# FRAME PARSER (EXTENDED FOR MULTI-BYTE)
# ==============================================================================

def process_decoded_bits(decoded_info):
    """
    Processes a binary string. Now handles both short and multi-byte standard frames.
    """
    bits = decoded_info
    if not bits or "[err" in bits:
        return f"Decoding failed or produced invalid bits: {bits}"

    # Handle implicit EOF for frames ending in a '1' (no final pulse)
    if len(bits) % 9 == 8 and bits[-1] == '1': # e.g. short frame is 8 bits before EOF
        bits += '0'
    elif (len(bits)-2) % 9 == 0 and bits[-1] == '1': # Standard frame
         bits += '0'


    # --- SHORT FRAME CHECK ---
    if len(bits) == 9:
        sof, data_bits, eof = bits[0], bits[1:8], bits[8]
        if sof != '0' or eof != '0':
            return f"Short Frame Error. Bits: {bits}"
        byte_val = int(data_bits[::-1], 2)
        hex_val = f"0x{byte_val:02X}"
        if byte_val == 0x26:
            return f"OK - Decoded: REQA ({hex_val})"
        if byte_val == 0x52:
            return f"OK - Decoded: WUPA ({hex_val})"
        return f"Decoded Unknown Short Frame: {hex_val}"

    # --- STANDARD FRAME CHECK ---
    if len(bits) > 9 and (len(bits) - 2) % 9 == 0:
        sof, frame_data, eof = bits[0], bits[1:-1], bits[-1]
        if sof != '0' or eof != '0':
            return f"Standard Frame Error. Bits: {bits}"

        num_bytes = len(frame_data) // 9
        decoded_bytes = []
        all_parity_ok = True

        for i in range(num_bytes):
            chunk = frame_data[i*9 : (i+1)*9]
            data_lsb, parity_bit = chunk[:8], chunk[8]

            # Odd Parity Check: if even number of 1s, parity must be 1.
            if (data_lsb.count('1') % 2 == 0) != (parity_bit == '1'):
                all_parity_ok = False
            
            byte_val = int(data_lsb[::-1], 2)
            decoded_bytes.append(f"0x{byte_val:02X}")
        
        parity_str = "OK" if all_parity_ok else "FAIL"
        return f"OK - Decoded Standard Frame: [{' '.join(decoded_bytes)}] Parity: {parity_str}"

    return f"Unknown or Partial Frame (len={len(bits)}): {bits}"

def parse_and_decode(log_data, discard_first_pulse=False):
    """Helper to parse and run the final decoder."""
    # ... (this function remains the same)
    pulse_trains = re.findall(r'pulses: \[(.*?)\]', log_data.replace('\n', ''))
    for train_str in pulse_trains:
        if not train_str.strip():
            continue
        try:
            pulses = [int(p.split(':')[1]) for p in train_str.strip().split()]
            if not pulses:
                continue
            if discard_first_pulse:
                pulses = pulses[1:]
            print(f"Analyzing Pulses: {pulses}")
            decoded_info = decode_pulses_to_bits(pulses)
            result = process_decoded_bits(decoded_info)
            print(f"  -> {result}\n")
        except Exception as e:
            print(f"Error processing segment '{train_str}': {e}")


# ==============================================================================
# TEST AND VALIDATION
# ==============================================================================
print("--- Decoding Ideal REQA ---")
reqa_log = "pulses: [0:128 1:192 2:128 3:192 4:192 5:192]"
parse_and_decode(reqa_log)

print("--- Decoding Ideal WUPA ---")
wupa_log = "pulses: [0:128 1:192 2:192 3:192 4:256]"
parse_and_decode(wupa_log)

print("--- Decoding Ideal Anticollision CL1 (0x93, 0x20) ---")
# This command is SOF + (0x93 + P) + (0x20 + P) + EOF
# 0x93 -> 10010011 -> 5 ones (odd) -> Parity = 0
# 0x20 -> 00100000 -> 1 one (odd) -> Parity = 0
# Bitstream: 0_110010010_000001000_0
# This produces 15 pulses after the SOF pulse.
ac_log = "pulses: [1:192 2:128 3:192 4:192 5:192 6:192 7:128 8:192 9:128 10:128 11:128 12:192 13:192 14:128 15:128]"
parse_and_decode(ac_log)

print("--- Decoding captures ---")
big_trains = []
big_trains.append("pulses: [18:198 19:129 20:129 21:129 22:129 23:193 24:257 25:257 26:193 27:129 28:129 29:129 30:129 31:129 32:129 33:193 34:129 35:129 36:128 37:256 38:257 39:193 40:193 41:257 42:129 43:193 44:193 45:129 46:194 ]")
big_trains.append("pulses: [48:175 49:129 50:193 51:193 52:193 53:257 ]")
big_trains.append("pulses: [65:196 66:193 67:129 68:193 69:193 70:193 71:193 72:129 73:193 74:129 75:129 76:129 77:193 78:193 79:129 80:129 ]")
big_trains.append("pulses: [104:199 105:193 106:129 107:193 108:193 109:193 110:193 111:129 112:193 113:129 114:129 115:193 116:128 117:128 118:193 119:129 120:129 121:129 122:193 123:193 124:129 125:193 126:129 127:193 128:193 129:193 130:128 131:128 132:129 133:129 134:129 135:129 136:129 137:129 138:193 139:257 140:257 141:129 142:256 143:128 144:129 145:193 146:193 147:257 148:193 149:129 150:129 151:129 152:193 153:257 154:256 155:128 156:128 157:193 158:193 159:257 160:257 161:193 162:129 163:129 164:129 165:128 166:128 167:128 ]")
big_trains.append("pulses: [183:199 184:193 185:256 186:257 187:193 188:193 189:129 190:193 191:129 192:129 193:129 194:193 195:193 196:129 197:129 ]")
big_trains.append("pulses: [223:198 224:193 225:257 226:256 227:193 228:193 229:129 230:193 231:129 232:129 233:193 234:129 235:129 236:193 237:128 238:193 239:257 240:129 241:129 242:129 243:193 244:193 245:257 246:193 247:129 248:193 249:128 250:128 251:128 252:129 253:257 254:257 255:193 256:193 257:193 258:129 259:129 260:129 261:129 262:128 263:192 264:193 265:129 266:193 267:257 268:193 269:129 270:193 271:193 272:129 273:129 274:192 275:129 276:129 277:193 278:193 279:257 280:129 281:129 282:129 283:129 284:257 ]")
big_trains.append("pulses: [300:196 301:130 302:129 303:129 304:129 305:193 306:129 307:193 308:193 309:193 310:129 311:129 312:129 313:128 314:128 315:128 316:193 317:257 318:193 319:129 320:129 321:129 322:129 323:129 324:129 325:129 326:129 327:193 328:256 329:257 330:193 ]")
big_trains.append("pulses: [414:111 415:128 416:128 417:129 418:129 419:193 420:257 421:257 422:193 423:129 424:129 425:129 426:129 427:129 428:128 429:193 430:129 431:129 432:129 433:257 434:257 435:193 436:193 437:257 438:129 439:192 440:192 441:129 442:194 ]")
big_trains.append("pulses: [444:146 445:129 446:193 447:193 448:193 449:257 ]")
big_trains.append("pulses: [461:199 462:193 463:129 464:193 465:193 466:193 467:193 468:128 469:192 470:128 471:129 472:193 473:129 474:129 475:193 476:129 477:129 478:129 479:193 480:193 481:129 482:192 483:128 484:193 485:193 486:193 487:129 488:129 489:129 490:129 491:129 492:129 493:129 494:129 495:193 496:258 497:257 498:129 499:257 500:129 501:129 502:193 503:193 504:257 505:193 506:129 507:129 508:129 509:193 510:257 511:257 ]")
for t in big_trains:
    parse_and_decode(t, discard_first_pulse=True)

'''
captures from version 4f751f8
pulses: []
pulses: [1:238 ]
pulses: [3:129 4:192 5:129 6:193 7:193 8:193 ]
pulses: [10:145 ]
pulses: [12:145 ]
pulses: [14:144 ]
pulses: [16:145 ]
pulses: [18:198 19:129 20:129 21:129 22:129 23:193 24:257 25:257 26:193 27:129 28:129 29:129 30:129 31:129 32:129 33:193 34:129 35:129 36:128 37:256 38:257 39:193 40:193 41:257 42:129 43:193 44:193 45:129 46:194 ]
pulses: [48:175 49:129 50:193 51:193 52:193 53:257 ]
pulses: [55:140 ]
pulses: [57:145 ]
pulses: [59:144 ]
pulses: [61:145 ]
pulses: [63:145 ]
pulses: [65:196 66:193 67:129 68:193 69:193 70:193 71:193 72:129 73:193 74:129 75:129 76:129 77:193 78:193 79:129 80:129 ]
pulses: [82:145 ]
pulses: [84:145 ]
pulses: [86:145 ]
pulses: [88:145 ]
pulses: [90:145 ]
pulses: [92:145 ]
pulses: [94:145 ]
pulses: [96:145 ]
pulses: [98:145 ]
pulses: [100:145 ]
pulses: [102:145 ]
pulses: [104:199 105:193 106:129 107:193 108:193 109:193 110:193 111:129 112:193 113:129 114:129 115:193 116:128 117:128 118:193 119:129 120:129 121:129 122:193 123:193 124:129 125:193 126:129 127:193 128:193 129:193 130:128 131:128 132:129 133:129 134:129 135:129 136:129 137:129 138:193 139:257 140:257 141:129 142:256 143:128 144:129 145:193 146:193 147:257 148:193 149:129 150:129 151:129 152:193 153:257 154:256 155:128 156:128 157:193 158:193 159:257 160:257 161:193 162:129 163:129 164:129 165:128 166:128 167:128 ]
pulses: [169:145 ]
pulses: [171:145 ]
pulses: [173:145 ]
pulses: [175:146 ]
pulses: [177:145 ]
pulses: [179:145 ]
pulses: [181:145 ]
pulses: [183:199 184:193 185:256 186:257 187:193 188:193 189:129 190:193 191:129 192:129 193:129 194:193 195:193 196:129 197:129 ]
pulses: [199:146 ]
pulses: [201:146 ]
pulses: [203:145 ]
pulses: [205:145 ]
pulses: [207:145 ]
pulses: [209:145 ]
pulses: [211:145 ]
pulses: [213:145 ]
pulses: [215:145 ]
pulses: [217:145 ]
pulses: [219:145 ]
pulses: [221:145 ]
pulses: [223:198 224:193 225:257 226:256 227:193 228:193 229:129 230:193 231:129 232:129 233:193 234:129 235:129 236:193 237:128 238:193 239:257 240:129 241:129 242:129 243:193 244:193 245:257 246:193 247:129 248:193 249:128 250:128 251:128 252:129 253:257 254:257 255:193 256:193 257:193 258:129 259:129 260:129 261:129 262:128 263:192 264:193 265:129 266:193 267:257 268:193 269:129 270:193 271:193 272:129 273:129 274:192 275:129 276:129 277:193 278:193 279:257 280:129 281:129 282:129 283:129 284:257 ]
pulses: [286:140 ]
pulses: [288:145 ]
pulses: [290:145 ]
pulses: [292:145 ]
pulses: [294:146 ]
pulses: [296:145 ]
pulses: [298:145 ]
pulses: [300:196 301:130 302:129 303:129 304:129 305:193 306:129 307:193 308:193 309:193 310:129 311:129 312:129 313:128 314:128 315:128 316:193 317:257 318:193 319:129 320:129 321:129 322:129 323:129 324:129 325:129 326:129 327:193 328:256 329:257 330:193 ]
pulses: [332:145 ]
pulses: [334:145 ]
pulses: [336:145 ]
pulses: [338:145 ]
pulses: [340:145 ]
pulses: [342:145 ]
pulses: [344:145 ]
pulses: [346:145 ]
pulses: [348:145 ]
pulses: [350:145 ]
pulses: [352:145 ]
pulses: [354:145 ]
pulses: [356:145 ]
pulses: [358:145 ]
pulses: [360:145 ]
pulses: [362:145 ]
pulses: [364:146 ]
pulses: [366:145 ]
pulses: [368:145 ]
pulses: [370:145 ]
pulses: [372:144 ]
pulses: [374:145 ]
pulses: [376:145 ]
pulses: [378:145 ]
pulses: [380:145 ]
pulses: [382:145 ]
pulses: [384:145 ]
pulses: [386:145 ]
pulses: [388:145 ]
pulses: [390:145 ]
pulses: [392:145 ]
pulses: [394:145 ]
pulses: [396:145 ]
pulses: [398:145 ]
pulses: [400:146 ]
pulses: [402:145 ]
pulses: [404:145 ]
pulses: [406:145 ]
pulses: [408:145 ]
pulses: [410:145 ]
pulses: [412:145 ]
pulses: [414:111 415:128 416:128 417:129 418:129 419:193 420:257 421:257 422:193 423:129 424:129 425:129 426:129 427:129 428:128 429:193 430:129 431:129 432:129 433:257 434:257 435:193 436:193 437:257 438:129 439:192 440:192 441:129 442:194 ]
pulses: [444:146 445:129 446:193 447:193 448:193 449:257 ]
pulses: [451:140 ]
pulses: [453:145 ]
pulses: [455:145 ]
pulses: [457:145 ]
pulses: [459:145 ]
pulses: [461:199 462:193 463:129 464:193 465:193 466:193 467:193 468:128 469:192 470:128 471:129 472:193 473:129 474:129 475:193 476:129 477:129 478:129 479:193 480:193 481:129 482:192 483:128 484:193 485:193 486:193 487:129 488:129 489:129 490:129 491:129 492:129 493:129 494:129 495:193 496:258 497:257 498:129 499:257 500:129 501:129 502:193 503:193 504:257 505:193 506:129 507:129 508:129 509:193 510:257 511:257 ]
'''