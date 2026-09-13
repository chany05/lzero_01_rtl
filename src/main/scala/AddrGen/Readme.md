# DMA Address Generator 테스트

## 구성

| 파일 | 역할 |
|---|---|
| `ref_model.py` | 독립 레퍼런스. 절대식으로 기대 시퀀스를 계산 |
| `TargetAddrGenSpec.scala` | 생성기 단위 테스트 |
| `DmaAddrGenSpec.scala` | FB + top 통합 테스트 |

## 검증 원리

RTL 은 주소를 **누적 가산**으로 만든다.

```
stride : tileAddr += strideTiles * 256
wrap   : tileAddr  = base + col * 256
```

레퍼런스는 **절대식**으로 만든다.

```
addr = base + (row * strideTiles + col) * 256 + msg * 64
```

두 결과가 일치하면 누적 로직에 드리프트가 없다는 뜻이다. 같은 계산을
Python 과 Scala 양쪽에 두어, 한쪽 구현 실수로 테스트가 통과해 버리는 일을
줄인다.

## 실행

```bash
python3 ref_model.py          # 레퍼런스 자체 점검 + 시퀀스 확인
python3 ref_model.py --dump   # 전체 주소 덤프

sbt test                                 # RTL 테스트 전체
sbt "testOnly dma.TargetAddrGenSpec"     # 생성기만
```

## 케이스 목록

### TargetAddrGenSpec

| 테스트 | 확인하는 것 |
|---|---|
| reference sequence (5 케이스) | transpose 방문 순서, 누적/절대 일치 |
| 4KB chunk 분할 | chunkLast 시점, tensorLast 조기 발생 없음 |
| short tail chunk | 4KB 미만 꼬리에서 두 플래그 동시 발생 |
| backpressure | ready 를 1/3 로 낮춰도 시퀀스 불변 |
| nPtr = 2 독립성 | W1/W2 포인터가 서로 간섭하지 않음 |
| restore 재생 | Fusion 되감기가 동일 시퀀스를 재현 |
| restore 로 halt 해제 | 명세서 4.3 강제 전환 경로 |
| load 중 abort | 발행 도중 load 시 정지 후 base 재시작 |

### FreqAddrGenSpec

| 테스트 | 확인하는 것 |
|---|---|
| Cache Mode | 선형 1회 후 tensorLast |
| 정확히 32KB | 경계값이 Cache 쪽으로 판정되는지 |
| Spill Mode | 되감기 발생, tensorLast 없음, 영역 이탈 없음 |

### DmaAddrGenSpec

| 테스트 | 확인하는 것 |
|---|---|
| 타겟별 라우팅 | UB/WB/PB 의 base·stride 매핑 |
| 타겟 교차 | 번갈아 trigger 해도 포인터 드리프트 없음 |
| fusion off | phase 고정 0, W2 미접근 |
| fusion on | intermNum 마다 W1/W2 교차 |
| UB 재읽기 | Phase 0/1 이 동일 영역 |
| 종료 게이팅 | Phase 0 경계는 강제 전환, Phase 1 경계가 진짜 끝 |

## 아직 안 덮는 것

- TileLink 프로토콜 준수 (irrevocable, alignment) - wrapper 붙인 뒤 `TLMonitor` 담당
- D channel 수신, source ID 관리
- Phase 가 타겟별로 어긋나는 상황. Arbiter 를 편향시켜 재현해야 함
- Write path