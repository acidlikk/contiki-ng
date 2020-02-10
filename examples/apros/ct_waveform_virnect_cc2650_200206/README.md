# APROS CT Waveform

- CC2650
- Virnect
- Waveform
  - 1024번 계측, 버퍼에 저장
  - 32번에 나눠 약 256ms마다 전송
  - header(16byte), data[32](2*32=64byte): 80byte 전송
  - 프로토콜 사용
  - data_fetch_ready(); (데이터 패킷에 대한 정보)
  - BS_170404 사용
  - 연속 동작, timer chip 안 씀