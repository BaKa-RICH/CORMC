# BASIC-02 BASIC Numeric Diagnostic

- status: `passed`
- actual_steps: `446` / max `900`
- expected APS case: `case_3`
- observed APS case: `case_3`
- active CVs: `B02_CLV`
- Eq.10 consumers: `none`
- merged past ramp: `True`

## Initial Vehicles

| vehicle_id | lane | road_role | x_global | y | v | type |
| --- | --- | --- | ---: | ---: | ---: | --- |
| B02_MV | on_ramp | on_ramp_mv | 6640.0 | -3.5 | 20.0 | cav |
| B02_CLV | lane_2 | mainline | 6654.0 | 0.0 | 20.0 | cav |
| B02_CFV | lane_2 | mainline | 6614.0 | 0.0 | 20.0 | cav |
| B02_TLV_CLV | lane_1 | mainline | 6663.0 | 3.5 | 15.0 | cav |

## Region

- first control-zone step: `5` at x `6650.5689447424`
- first merge-zone step: `323` at x `6950.166586995788`
- illegal pre-control events: `0`

## Timelines

- first APS: `step=5, reason=first_aps, aps_case=case_3, clv_id=B02_CLV, cfv_id=B02_CFV`
- APS excluded candidates: `none`
- first cached boundary invalidation: `none`
- CUC choices: `5:stay_lane_2; 6:stay_lane_2; 7:stay_lane_2; 8:stay_lane_2; 9:stay_lane_2; 10:stay_lane_2; 11:stay_lane_2; 12:stay_lane_2; 13:stay_lane_2; 14:stay_lane_2; 15:stay_lane_2; 16:stay_lane_2; 17:stay_lane_2; 18:stay_lane_2; 19:stay_lane_2; 20:stay_lane_2; 21:stay_lane_2; 22:stay_lane_2; 23:stay_lane_2; 24:stay_lane_2; 25:stay_lane_2; 26:stay_lane_2; 27:stay_lane_2; 28:stay_lane_2; 29:stay_lane_2; 30:stay_lane_2; 31:stay_lane_2; 32:stay_lane_2; 33:stay_lane_2; 34:stay_lane_2; 35:stay_lane_2; 36:stay_lane_2; 37:stay_lane_2; 38:stay_lane_2; 39:stay_lane_2; 40:stay_lane_2; 41:stay_lane_2; 42:stay_lane_2; 43:stay_lane_2; 44:stay_lane_2; 45:stay_lane_2; 46:stay_lane_2; 47:stay_lane_2; 48:stay_lane_2; 49:stay_lane_2; 50:stay_lane_2; 51:stay_lane_2; 52:stay_lane_2; 53:stay_lane_2; 54:stay_lane_2; 55:stay_lane_2; 56:stay_lane_2; 57:stay_lane_2; 58:stay_lane_2; 59:stay_lane_2; 60:stay_lane_2; 61:stay_lane_2; 62:stay_lane_2; 63:stay_lane_2; 64:stay_lane_2; 65:stay_lane_2; 66:stay_lane_2; 67:stay_lane_2; 68:stay_lane_2; 69:stay_lane_2; 70:stay_lane_2; 71:stay_lane_2; 72:stay_lane_2; 73:stay_lane_2; 74:stay_lane_2; 75:stay_lane_2; 76:stay_lane_2; 77:stay_lane_2; 78:stay_lane_2; 79:stay_lane_2; 80:stay_lane_2; 81:stay_lane_2; 82:stay_lane_2; 83:stay_lane_2; 84:stay_lane_2; 85:stay_lane_2; 86:stay_lane_2; 87:stay_lane_2; 88:stay_lane_2; 89:stay_lane_2; 90:stay_lane_2; 91:stay_lane_2; 92:stay_lane_2; 93:stay_lane_2; 94:stay_lane_2; 95:stay_lane_2; 96:stay_lane_2; 97:stay_lane_2; 98:stay_lane_2; 99:stay_lane_2; 100:stay_lane_2; 101:stay_lane_2; 102:stay_lane_2; 103:stay_lane_2; 104:stay_lane_2; 105:stay_lane_2; 106:stay_lane_2; 107:stay_lane_2; 108:stay_lane_2; 109:stay_lane_2; 110:stay_lane_2; 111:stay_lane_2; 112:stay_lane_2; 113:stay_lane_2; 114:stay_lane_2; 115:stay_lane_2; 116:stay_lane_2; 117:stay_lane_2; 118:stay_lane_2; 119:stay_lane_2; 120:stay_lane_2; 121:stay_lane_2; 122:stay_lane_2; 123:stay_lane_2; 124:stay_lane_2; 125:stay_lane_2; 126:stay_lane_2; 127:stay_lane_2; 128:stay_lane_2; 129:stay_lane_2; 130:stay_lane_2; 131:stay_lane_2; 132:stay_lane_2; 133:stay_lane_2; 134:stay_lane_2; 135:stay_lane_2; 136:stay_lane_2; 137:stay_lane_2; 138:stay_lane_2; 139:stay_lane_2; 140:stay_lane_2; 141:stay_lane_2; 142:stay_lane_2; 143:stay_lane_2; 144:stay_lane_2; 145:stay_lane_2; 146:stay_lane_2; 147:stay_lane_2; 148:stay_lane_2; 149:stay_lane_2; 150:stay_lane_2; 151:stay_lane_2; 152:stay_lane_2; 153:stay_lane_2; 154:stay_lane_2; 155:stay_lane_2; 156:stay_lane_2; 157:stay_lane_2; 158:stay_lane_2; 159:stay_lane_2; 160:stay_lane_2; 161:stay_lane_2; 162:stay_lane_2; 163:stay_lane_2; 164:stay_lane_2; 165:stay_lane_2; 166:stay_lane_2; 167:stay_lane_2; 168:stay_lane_2; 169:stay_lane_2; 170:stay_lane_2; 171:stay_lane_2; 172:stay_lane_2; 173:stay_lane_2; 174:stay_lane_2; 175:stay_lane_2; 176:stay_lane_2; 177:stay_lane_2; 178:stay_lane_2; 179:stay_lane_2; 180:stay_lane_2; 181:stay_lane_2; 182:stay_lane_2; 183:stay_lane_2; 184:stay_lane_2; 185:stay_lane_2; 186:stay_lane_2; 187:stay_lane_2; 188:stay_lane_2; 189:stay_lane_2; 190:stay_lane_2; 191:stay_lane_2; 192:stay_lane_2; 193:stay_lane_2; 194:stay_lane_2; 195:stay_lane_2; 196:stay_lane_2; 197:stay_lane_2; 198:stay_lane_2; 199:stay_lane_2; 200:stay_lane_2; 201:stay_lane_2; 202:stay_lane_2; 203:stay_lane_2; 204:stay_lane_2; 205:stay_lane_2; 206:stay_lane_2; 207:stay_lane_2; 208:stay_lane_2; 209:stay_lane_2; 210:stay_lane_2; 211:stay_lane_2; 212:stay_lane_2; 213:stay_lane_2; 214:stay_lane_2; 215:stay_lane_2; 216:stay_lane_2; 217:stay_lane_2; 218:stay_lane_2; 219:stay_lane_2; 220:stay_lane_2; 221:stay_lane_2; 222:stay_lane_2; 223:stay_lane_2; 224:stay_lane_2; 225:stay_lane_2; 226:stay_lane_2; 227:stay_lane_2; 228:stay_lane_2; 229:stay_lane_2; 230:stay_lane_2; 231:stay_lane_2; 232:stay_lane_2; 233:stay_lane_2; 234:stay_lane_2; 235:stay_lane_2; 236:stay_lane_2; 237:stay_lane_2; 238:stay_lane_2; 239:stay_lane_2; 240:stay_lane_2; 241:stay_lane_2; 242:stay_lane_2; 243:stay_lane_2; 244:stay_lane_2; 245:stay_lane_2; 246:stay_lane_2; 247:stay_lane_2; 248:stay_lane_2; 249:stay_lane_2; 250:stay_lane_2; 251:stay_lane_2; 252:stay_lane_2; 253:stay_lane_2; 254:stay_lane_2; 255:stay_lane_2; 256:stay_lane_2; 257:stay_lane_2; 258:stay_lane_2; 259:stay_lane_2; 260:stay_lane_2; 261:stay_lane_2; 262:stay_lane_2; 263:stay_lane_2; 264:stay_lane_2; 265:stay_lane_2; 266:stay_lane_2; 267:stay_lane_2; 268:stay_lane_2; 269:stay_lane_2; 270:stay_lane_2; 271:stay_lane_2; 272:stay_lane_2; 273:stay_lane_2; 274:stay_lane_2; 275:stay_lane_2; 276:stay_lane_2; 277:stay_lane_2; 278:stay_lane_2; 279:stay_lane_2; 280:stay_lane_2; 281:stay_lane_2; 282:stay_lane_2; 283:stay_lane_2; 284:stay_lane_2; 285:stay_lane_2; 286:stay_lane_2; 287:stay_lane_2; 288:stay_lane_2; 289:stay_lane_2; 290:stay_lane_2; 291:stay_lane_2; 292:stay_lane_2; 293:stay_lane_2; 294:stay_lane_2; 295:stay_lane_2; 296:stay_lane_2; 297:stay_lane_2; 298:stay_lane_2; 299:stay_lane_2; 300:stay_lane_2; 301:stay_lane_2; 302:stay_lane_2; 303:stay_lane_2; 304:stay_lane_2; 305:stay_lane_2; 306:stay_lane_2; 307:stay_lane_2; 308:stay_lane_2; 309:stay_lane_2; 310:stay_lane_2; 311:stay_lane_2; 312:stay_lane_2; 313:stay_lane_2; 314:stay_lane_2; 315:stay_lane_2; 316:stay_lane_2; 317:stay_lane_2; 318:stay_lane_2; 319:stay_lane_2; 320:stay_lane_2; 321:stay_lane_2; 322:stay_lane_2; 323:stay_lane_2`
- assignment validity: `323:False`
- assignment invalid reasons: `323:wrong_order`
- Eq.53: `323:True`
- merge states: `0:not_started; 324:executing; 374:merged`

## Issues

### Resolved

- `BASIC-02:region:pre_control_suppression_fixed`: Pre-control module suppression held until MV entered control_zone.

### Unresolved

- `BASIC-02:sanity:0` [recording_issue]: assignment_invalid=warning: wrong_order

## Artifacts

- trajectory: `artifacts\basic\basic02_900_assignment_lifecycle_check\BASIC-02\trajectory.csv`
- events: `artifacts\basic\basic02_900_assignment_lifecycle_check\BASIC-02\events.jsonl`
- sanity: `artifacts\basic\basic02_900_assignment_lifecycle_check\BASIC-02\sanity.jsonl`
- numeric_summary: `artifacts\basic\basic02_900_assignment_lifecycle_check\BASIC-02\numeric_summary.json`
