#include <SimpleModbusSlave.h>

// --- Pinos do Controle Físico ---
#define POT_PIN    A0  // Pino analógico para o potenciômetro      // <-- NOVO
#define BUTTON_PIN 2   // Pino digital para o botão de confirmação // <-- NOVO

// --- Modbus ---
#define TOTAL_REGS_SIZE 1
unsigned int holdingRegs[TOTAL_REGS_SIZE]; // registradores Modbus

// --- PWM ---
#define PWM_PIN  9     // OC1A, pino D9 do Uno
#define PWM_TOP  511   // ICR1 = 511 → fPWM ≈ 31.25 kHz

void setup() {
  // --- Modbus RTU Slave via USB Serial ---
  modbus_configure(9600, 1, 0, TOTAL_REGS_SIZE, 0);
  holdingRegs[0] = 0; // Inicia com potência zero

  // --- Configura pinos de entrada para o controle físico ---
  pinMode(BUTTON_PIN, INPUT);                               // <-- NOVO

  // --- Timer1 Fast-PWM no pino D9 (OC1A) ---
  pinMode(PWM_PIN, OUTPUT);
  TCCR1A = (1 << COM1A1) | (1 << WGM11);
  TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS10);
  ICR1   = PWM_TOP;
}

void loop() {
  // 1. Responde a pedidos Modbus e atualiza holdingRegs[0] se o SCADA escrever.
  modbus_update(holdingRegs);

  // 2. Verifica o controle físico (potenciômetro e botão)     // <-- INÍCIO DO NOVO BLOCO LÓGICO
  if (digitalRead(BUTTON_PIN) == HIGH) {
    // Botão foi pressionado, vamos ler o potenciômetro.
    int potValue = analogRead(POT_PIN); // Lê o valor de 0 a 1023

    // Mapeia o valor do potenciômetro (0-1023) para a faixa de controle (0-255)
    // que o SCADA utiliza.
    unsigned int newPowerValue = map(potValue, 0, 1023, 0, 255);

    // ATUALIZA O REGISTRADOR MODBUS! Este é o ponto chave.
    holdingRegs[0] = newPowerValue;

    // Pequeno delay para "debounce", evitar múltiplas leituras em um aperto.
    delay(200); 
  }
  // <-- FIM DO NOVO BLOCO LÓGICO

  // 3. Atualiza a saída PWM com o valor que estiver em holdingRegs[0],
  //    seja ele vindo do SCADA ou do controle físico.
  //    Esta linha é a original, não precisa mudar.
  OCR1A = (unsigned long)holdingRegs[0] * PWM_TOP / 255UL;
}