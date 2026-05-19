# SAPF STM32F407 — Single-Phase Active Power Filter
# Tek Fazlı Aktif Güç Filtresi

---

## English

### What This Project Is
A single-phase shunt active power filter (SAPF) implemented on
STM32F407VGT6 Discovery board. The APF connects in parallel with
a nonlinear load on a 48V AC grid and injects harmonic current
with opposite sign to cancel harmonic pollution. The result is a
clean sinusoidal source current.

The control algorithm was validated in MATLAB Simulink before
being implemented on the STM32F407.

### Authors
- Özcan YARDIMCI — Sakarya University, Electrical Electronics Engineering
- Yusuf Cafer TOK — Sakarya University, Electrical Electronics Engineering

### Hardware
| Component | Value | Status |
|---|---|---|
| MCU | STM32F407VGT6 Discovery | Confirmed |
| Grid | 48V RMS transformer secondary | Confirmed |
| DC link setpoint | 90V | Confirmed |
| APF inductor | 4.21mH air-core | Confirmed |
| Current sensors | 2× ACS712-05B (5A) | Confirmed |
| Gate drivers | 2× IR2103 | Confirmed |
| MOSFETs | 4× IRF3710 | Confirmed |
| Snubber capacitor | 4.7µF 400V | Confirmed |
| Load (test) | Diode bridge + resistor | Confirmed |
| DC link capacitor | TBD | Pending |
| V_S voltage divider | TBD | Pending |
| V_dc voltage divider | TBD | Pending |
| ACS712 output divider | TBD | Pending |
| Gate resistors | TBD | Pending |
| Bootstrap capacitors | TBD | Pending |
| Diode bridge rating | TBD | Pending |

### Pin Map
| Signal | Pin | Type | Full Scale |
|---|---|---|---|
| I_APF | PA1 | Bipolar | ±5A |
| V_dc | PA2 | Unipolar | 0-100V |
| V_S | PA3 | Bipolar | ±70V |
| I_L1 | PC1 | Bipolar | ±5A |
| gate_APF | PA8 | TIM1_CH1 | — |
| gate_bar | PB13 | TIM1_CH1N | — |
| ADC trigger | — | TIM1_CH2 fixed | CCR2=4199 |

### Control Algorithm
- Outer loop: APF_RefGen — reference current generator
  - Instantaneous power theory
  - 5Hz IIR low-pass filter for average active power
  - PID controller for DC link voltage regulation (90V)
- Inner loop: APF_HystCtrl — hysteresis current controller
  - Hysteresis band: 0.5A
  - Switching frequency: ~20kHz

### Repository Structure

SAPF_STM32F407/
├── firmware/            STM32CubeIDE project
├── simulink/            Validated Simulink model
├── signal_generator/    ESP32 test signal generator
├── hardware/
│   ├── schematic/       Circuit schematic
│   └── bom/             Bill of materials
└── docs/
├── datasheets/      Component datasheets
└── papers/          Reference papers


---

## Türkçe

### Proje Nedir
STM32F407VGT6 Discovery kartı üzerinde gerçekleştirilen tek fazlı
paralel aktif güç filtresi (SAPF) projesidir. APF, 48V AC şebekede
doğrusal olmayan yük ile paralel bağlanır ve harmonik kirliliği
iptal etmek için zıt işaretli harmonik akım enjekte eder. Sonuç
olarak kaynak akımı temiz sinüzoidal forma kavuşur.

Kontrol algoritması, STM32F407 üzerinde uygulanmadan önce MATLAB
Simulink ortamında doğrulanmıştır.

### Yazarlar
- Özcan YARDIMCI — Sakarya Üniversitesi, Elektrik Elektronik Mühendisliği
- Yusuf Cafer TOK — Sakarya Üniversitesi, Elektrik Elektronik Mühendisliği

### Donanım
| Bileşen | Değer | Durum |
|---|---|---|
| MCU | STM32F407VGT6 Discovery | Onaylandı |
| Şebeke | 48V RMS trafo sekonderi | Onaylandı |
| DC bara set noktası | 90V | Onaylandı |
| APF endüktansı | 4.21mH hava nüveli | Onaylandı |
| Akım sensörleri | 2× ACS712-05B (5A) | Onaylandı |
| Gate sürücüleri | 2× IR2103 | Onaylandı |
| MOSFET | 4× IRF3710 | Onaylandı |
| Snubber kondansatör | 4.7µF 400V | Onaylandı |
| Yük (test) | Diyot köprüsü + direnç | Onaylandı |
| DC bara kondansatörü | Belirlenecek | Beklemede |
| V_S gerilim bölücü | Belirlenecek | Beklemede |
| V_dc gerilim bölücü | Belirlenecek | Beklemede |
| ACS712 çıkış bölücü | Belirlenecek | Beklemede |
| Gate dirençleri | Belirlenecek | Beklemede |
| Bootstrap kondansatörler | Belirlenecek | Beklemede |
| Diyot köprüsü akım değeri | Belirlenecek | Beklemede |

### Pin Haritası
| Sinyal | Pin | Tür | Tam Ölçek |
|---|---|---|---|
| I_APF | PA1 | Bipolar | ±5A |
| V_dc | PA2 | Unipolar | 0-100V |
| V_S | PA3 | Bipolar | ±70V |
| I_L1 | PC1 | Bipolar | ±5A |
| gate_APF | PA8 | TIM1_CH1 | — |
| gate_bar | PB13 | TIM1_CH1N | — |
| ADC tetik | — | TIM1_CH2 sabit | CCR2=4199 |

### Kontrol Algoritması
- Dış döngü: APF_RefGen — referans akım üreteci
  - Anlık güç teorisi
  - Ortalama aktif güç için 5Hz IIR alçak geçiren filtre
  - DC bara gerilim regülasyonu için PID kontrolör (90V)
- İç döngü: APF_HystCtrl — histerezis akım kontrolörü
  - Histerezis bandı: 0.5A
  - Anahtarlama frekansı: ~20kHz

### Depo Yapısı

SAPF_STM32F407/
├── firmware/            STM32CubeIDE projesi
├── simulink/            Doğrulanmış Simulink modeli
├── signal_generator/    ESP32 test sinyal üreteci
├── hardware/
│   ├── schematic/       Devre şeması
│   └── bom/             Malzeme listesi
└── docs/
├── datasheets/      Bileşen veri sayfaları
└── papers/          Referans makaleler