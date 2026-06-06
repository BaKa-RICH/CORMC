# BASIC-02 BASIC Numeric Diagnostic

- status: `passed`
- actual_steps: `221` / max `900`
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
| B02_CFV | lane_2 | mainline | 6560.0 | 0.0 | 20.0 | cav |
| B02_TLV_CLV | lane_1 | mainline | 6663.0 | 3.5 | 15.0 | cav |

## Region

- first control-zone step: `5` at x `6650.5689447424`
- first merge-zone step: `121` at x `6950.965157230497`
- illegal pre-control events: `0`

## Timelines

- first APS: `step=5, reason=first_aps, aps_case=case_3, clv_id=B02_CLV, cfv_id=B02_CFV`
- APS excluded candidates: `none`
- first cached boundary invalidation: `none`
- CUC choices: `5:stay_lane_2; 6:stay_lane_2; 7:stay_lane_2; 8:stay_lane_2; 9:stay_lane_2; 10:stay_lane_2; 11:stay_lane_2; 12:stay_lane_2; 13:stay_lane_2; 14:stay_lane_2; 15:stay_lane_2; 16:stay_lane_2; 17:stay_lane_2; 18:stay_lane_2; 19:stay_lane_2; 20:stay_lane_2; 21:stay_lane_2; 22:stay_lane_2; 23:stay_lane_2; 24:stay_lane_2; 25:stay_lane_2; 26:stay_lane_2; 27:stay_lane_2; 28:stay_lane_2; 29:stay_lane_2; 30:stay_lane_2; 31:stay_lane_2; 32:stay_lane_2; 33:stay_lane_2; 34:stay_lane_2; 35:stay_lane_2; 36:stay_lane_2; 37:stay_lane_2; 38:stay_lane_2; 39:stay_lane_2; 40:stay_lane_2; 41:stay_lane_2; 42:stay_lane_2; 43:stay_lane_2; 44:stay_lane_2; 45:stay_lane_2; 46:stay_lane_2; 47:stay_lane_2; 48:stay_lane_2; 49:stay_lane_2; 50:stay_lane_2; 51:stay_lane_2; 52:stay_lane_2; 53:stay_lane_2; 54:stay_lane_2; 55:stay_lane_2; 56:stay_lane_2; 57:stay_lane_2; 58:stay_lane_2; 59:stay_lane_2; 60:stay_lane_2; 61:stay_lane_2; 62:stay_lane_2; 63:stay_lane_2; 64:stay_lane_2; 65:stay_lane_2; 66:stay_lane_2; 67:stay_lane_2; 68:stay_lane_2; 69:stay_lane_2; 70:stay_lane_2; 71:stay_lane_2; 72:stay_lane_2; 73:stay_lane_2; 74:stay_lane_2; 75:stay_lane_2; 76:stay_lane_2; 77:stay_lane_2; 78:stay_lane_2; 79:stay_lane_2; 80:stay_lane_2; 81:stay_lane_2; 82:stay_lane_2; 83:stay_lane_2; 84:stay_lane_2; 85:stay_lane_2; 86:stay_lane_2; 87:stay_lane_2; 88:stay_lane_2; 89:stay_lane_2; 90:stay_lane_2; 91:stay_lane_2; 92:stay_lane_2; 93:stay_lane_2; 94:stay_lane_2; 95:stay_lane_2; 96:stay_lane_2; 97:stay_lane_2; 98:stay_lane_2; 99:stay_lane_2; 100:stay_lane_2; 101:stay_lane_2; 102:stay_lane_2; 103:stay_lane_2; 104:stay_lane_2; 105:stay_lane_2; 106:stay_lane_2; 107:stay_lane_2; 108:stay_lane_2; 109:stay_lane_2; 110:stay_lane_2; 111:stay_lane_2; 112:stay_lane_2; 113:stay_lane_2; 114:stay_lane_2; 115:stay_lane_2; 116:stay_lane_2; 117:stay_lane_2; 118:stay_lane_2; 119:stay_lane_2; 120:stay_lane_2; 121:stay_lane_2; 122:stay_lane_2; 123:stay_lane_2; 124:stay_lane_2; 125:stay_lane_2; 126:stay_lane_2; 127:stay_lane_2; 128:stay_lane_2; 129:stay_lane_2; 130:stay_lane_2; 131:stay_lane_2; 132:stay_lane_2; 133:stay_lane_2; 134:stay_lane_2; 135:stay_lane_2; 136:stay_lane_2; 137:stay_lane_2; 138:stay_lane_2; 139:stay_lane_2; 140:stay_lane_2; 141:stay_lane_2; 142:stay_lane_2; 143:stay_lane_2; 144:stay_lane_2; 145:stay_lane_2; 146:stay_lane_2; 147:stay_lane_2; 148:stay_lane_2; 149:stay_lane_2; 150:stay_lane_2; 151:stay_lane_2; 152:stay_lane_2; 153:stay_lane_2; 154:stay_lane_2; 155:stay_lane_2; 156:stay_lane_2; 157:stay_lane_2; 158:stay_lane_2; 159:stay_lane_2; 160:stay_lane_2; 161:stay_lane_2; 162:stay_lane_2; 163:stay_lane_2; 164:stay_lane_2; 165:stay_lane_2; 166:stay_lane_2; 167:stay_lane_2; 168:stay_lane_2; 169:stay_lane_2; 170:stay_lane_2; 171:stay_lane_2; 172:stay_lane_2; 173:stay_lane_2; 174:stay_lane_2; 175:stay_lane_2; 176:stay_lane_2; 177:stay_lane_2; 178:stay_lane_2; 179:stay_lane_2; 180:stay_lane_2; 181:stay_lane_2; 182:stay_lane_2; 183:stay_lane_2; 184:stay_lane_2; 185:stay_lane_2; 186:stay_lane_2; 187:stay_lane_2; 188:stay_lane_2; 189:stay_lane_2; 190:stay_lane_2; 191:stay_lane_2; 192:stay_lane_2; 193:stay_lane_2; 194:stay_lane_2; 195:stay_lane_2; 196:stay_lane_2; 197:stay_lane_2; 198:stay_lane_2; 199:stay_lane_2; 200:stay_lane_2; 201:stay_lane_2; 202:stay_lane_2; 203:stay_lane_2; 204:stay_lane_2; 205:stay_lane_2; 206:stay_lane_2; 207:stay_lane_2; 208:stay_lane_2; 209:stay_lane_2; 210:stay_lane_2; 211:stay_lane_2; 212:stay_lane_2; 213:stay_lane_2; 214:stay_lane_2; 215:stay_lane_2; 216:stay_lane_2; 217:stay_lane_2; 218:stay_lane_2; 219:stay_lane_2; 220:stay_lane_2`
- MV longitudinal relation: `0:None; 1:None; 2:None; 3:None; 4:None; 5:B02_CLV; 6:B02_CLV; 7:B02_CLV; 8:B02_CLV; 9:B02_CLV; 10:B02_CLV; 11:B02_CLV; 12:B02_CLV; 13:B02_CLV; 14:B02_CLV; 15:B02_CLV; 16:B02_CLV; 17:B02_CLV; 18:B02_CLV; 19:B02_CLV; 20:B02_CLV; 21:B02_CLV; 22:B02_CLV; 23:B02_CLV; 24:B02_CLV; 25:B02_CLV; 26:B02_CLV; 27:B02_CLV; 28:B02_CLV; 29:B02_CLV; 30:B02_CLV; 31:B02_CLV; 32:B02_CLV; 33:B02_CLV; 34:B02_CLV; 35:B02_CLV; 36:B02_CLV; 37:B02_CLV; 38:B02_CLV; 39:B02_CLV; 40:B02_CLV; 41:B02_CLV; 42:B02_CLV; 43:B02_CLV; 44:B02_CLV; 45:B02_CLV; 46:B02_CLV; 47:B02_CLV; 48:B02_CLV; 49:B02_CLV; 50:B02_CLV; 51:B02_CLV; 52:B02_CLV; 53:B02_CLV; 54:B02_CLV; 55:B02_CLV; 56:B02_CLV; 57:B02_CLV; 58:B02_CLV; 59:B02_CLV; 60:B02_CLV; 61:B02_CLV; 62:B02_CLV; 63:B02_CLV; 64:B02_CLV; 65:B02_CLV; 66:B02_CLV; 67:B02_CLV; 68:B02_CLV; 69:B02_CLV; 70:B02_CLV; 71:B02_CLV; 72:B02_CLV; 73:B02_CLV; 74:B02_CLV; 75:B02_CLV; 76:B02_CLV; 77:B02_CLV; 78:B02_CLV; 79:B02_CLV; 80:B02_CLV; 81:B02_CLV; 82:B02_CLV; 83:B02_CLV; 84:B02_CLV; 85:B02_CLV; 86:B02_CLV; 87:B02_CLV; 88:B02_CLV; 89:B02_CLV; 90:B02_CLV; 91:B02_CLV; 92:B02_CLV; 93:B02_CLV; 94:B02_CLV; 95:B02_CLV; 96:B02_CLV; 97:B02_CLV; 98:B02_CLV; 99:B02_CLV; 100:B02_CLV; 101:B02_CLV; 102:B02_CLV; 103:B02_CLV; 104:B02_CLV; 105:B02_CLV; 106:B02_CLV; 107:B02_CLV; 108:B02_CLV; 109:B02_CLV; 110:B02_CLV; 111:B02_CLV; 112:B02_CLV; 113:B02_CLV; 114:B02_CLV; 115:B02_CLV; 116:B02_CLV; 117:B02_CLV; 118:B02_CLV; 119:B02_CLV; 120:B02_CLV; 121:B02_CLV; 122:B02_CLV; 123:B02_CLV; 124:B02_CLV; 125:B02_CLV; 126:B02_CLV; 127:B02_CLV; 128:B02_CLV; 129:B02_CLV; 130:B02_CLV; 131:B02_CLV; 132:B02_CLV; 133:B02_CLV; 134:B02_CLV; 135:B02_CLV; 136:B02_CLV; 137:B02_CLV; 138:B02_CLV; 139:B02_CLV; 140:B02_CLV; 141:B02_CLV; 142:B02_CLV; 143:B02_CLV; 144:B02_CLV; 145:B02_CLV; 146:B02_CLV; 147:B02_CLV; 148:B02_CLV; 149:B02_CLV; 150:B02_CLV; 151:B02_CLV; 152:B02_CLV; 153:B02_CLV; 154:B02_CLV; 155:B02_CLV; 156:B02_CLV; 157:B02_CLV; 158:B02_CLV; 159:B02_CLV; 160:B02_CLV; 161:B02_CLV; 162:B02_CLV; 163:B02_CLV; 164:B02_CLV; 165:B02_CLV; 166:B02_CLV; 167:B02_CLV; 168:B02_CLV; 169:B02_CLV; 170:B02_CLV; 171:B02_CLV; 172:B02_CLV; 173:B02_CLV; 174:B02_CLV; 175:B02_CLV; 176:B02_CLV; 177:B02_CLV; 178:B02_CLV; 179:B02_CLV; 180:B02_CLV; 181:B02_CLV; 182:B02_CLV; 183:B02_CLV; 184:B02_CLV; 185:B02_CLV; 186:B02_CLV; 187:B02_CLV; 188:B02_CLV; 189:B02_CLV; 190:B02_CLV; 191:B02_CLV; 192:B02_CLV; 193:B02_CLV; 194:B02_CLV; 195:B02_CLV; 196:B02_CLV; 197:B02_CLV; 198:B02_CLV; 199:B02_CLV; 200:B02_CLV; 201:B02_CLV; 202:B02_CLV; 203:B02_CLV; 204:B02_CLV; 205:B02_CLV; 206:B02_CLV; 207:B02_CLV; 208:B02_CLV; 209:B02_CLV; 210:B02_CLV; 211:B02_CLV; 212:B02_CLV; 213:B02_CLV; 214:B02_CLV; 215:B02_CLV; 216:B02_CLV; 217:B02_CLV; 218:B02_CLV; 219:B02_CLV; 220:B02_CLV`
- assignment validity: `121:True`
- assignment invalid reasons: `121:None`
- Eq.53: `121:True`
- merge states: `0:not_started; 122:executing; 152:merged`

## Issues

### Resolved

- `BASIC-02:region:pre_control_suppression_fixed`: Pre-control module suppression held until MV entered control_zone.

### Unresolved

- none

## Artifacts

- trajectory: `artifacts\basic\basic02_case3_mv_clv_relation_900\BASIC-02\trajectory.csv`
- events: `artifacts\basic\basic02_case3_mv_clv_relation_900\BASIC-02\events.jsonl`
- sanity: `artifacts\basic\basic02_case3_mv_clv_relation_900\BASIC-02\sanity.jsonl`
- time_space_png: `artifacts\basic\basic02_case3_mv_clv_relation_900\BASIC-02\time_space.png`
- numeric_summary: `artifacts\basic\basic02_case3_mv_clv_relation_900\BASIC-02\numeric_summary.json`
