# Frame codes
## Encoding
- 1 byte -> flight controller
- 1 byte -> frame code
- 1 byte -> optical flow
- 1 byte -> customization

## Codes in use

### Copter

| byte | flight controller | frame | OF | customization |
| --- | --- | --- | --- | --- |
| `0x01010001` | AP3  | M460  | no  | none |
| `0x01020001` | AP3  | M490  | no  | none |
| `0x01030001` | AP3  | M6T22 | no  | none |
| `0x02010001` | AP5  | M460  | no  | none |
| `0x02020001` | AP5  | M490  | no  | none |
| `0x03030001` | AP6  | M6T22 | no  | none |
| `0x03010001` | AP6  | M460  | no  | none |
| `0x03010002` | AP6  | M460  | no  | A10  |
| `0x03020001` | AP6  | M490  | no  | none |
| `0x03020002` | AP6  | M490  | no  | A10  |
| `0x03030001` | AP6  | M6T22 | no  | none |
| `0x03040001` | AP6  | M4T12 | no  | none |      
| `0x03040002` | AP6  | M4T12 | no  | RTK  |
| `0x03050001` | AP6  | M4P   | no  | none |
| `0x03060001` | AP6  | M450  | no  | none |
| `0x03060101` | AP6  | M450  | yes | none |
| `0x03060002` | AP6  | M450  | no  | A10  |
| `0x03060102` | AP6  | M450  | yes | A10  |
| `0x03060003` | AP6  | M450  | no  | no GPS |
| `0x03400001` | AP6  | E1    | no  | none |         
| `0x03410001` | AP6  | TB70  | no  | none |         
| `0x03420001` | AP6  | E2    | no  | none |         
| `0x03440001` | AP6  | E1v2  | no  | none |         
| `0x04070001` | AP6m | M3    | no  | none |
| `0x04070101` | AP6m | M3    | yes | none |
| `0x04070002` | AP6m | M3    | no  | A10  |

### Rover

| byte | flight controller | frame | payload | customization |
| --- | --- | --- | --- | --- |
| `0x01800000` | AP3    | GA22  | none | developers |
| `0x01800001` | AP3    | GA22  | none | none |
| `0x01810000` | AP3    | GA45  | none | developers |
| `0x01810001` | AP3    | GA45  | none | none |
| `0x02800000` | AP6    | GA22  | none | developers |
| `0x02800001` | AP6    | GA22  | none | none |
| `0x02810000` | AP6    | GA45  | none | developers |
| `0x02810001` | AP6    | GA45  | none | none |
| `0x03800000` | AP6v2  | GA22  | none | developers |
| `0x03800001` | AP6v2  | GA22  | none | none |
| `0x03810000` | AP6v2  | GA45  | none | developers |
| `0x03810001` | AP6v2  | GA45  | none | none |


