#include <SimpleModbusSlave.h>
#include <PID_v1.h> // Inclui a biblioteca PID

// --- Pinos de Controle e Feedback ---
#define PWM_PIN       9   // Saída de potência para o cooler
#define RPM_PIN       2   // Entrada do sinal de RPM (precisa ser pino de interrupção)
#define POT_PIN       A0  // Entrada do potenciômetro
#define BUTTON_PIN    4   // Entrada do botão físico (movido para D4)

// --- Variáveis para Medição de RPM ---
// 'volatile' é essencial para variáveis usadas dentro e fora de interrupções
volatile unsigned long pulse_count = 0; 
unsigned long last_rpm_read_time = 0;
double current_rpm = 0;

// --- Configuração do Modbus ---
#define TOTAL_REGS_SIZE 2
// holdingRegs[0] -> Setpoint de RPM (lido/escrito pelo SCADA)
// holdingRegs[1] -> RPM Atual (lido pelo SCADA)
unsigned int holdingRegs[TOTAL_REGS_SIZE];

// --- Configuração do PID ---
double setpoint_rpm, pwm_output; // Variáveis que o PID usa
// Estes valores de Kp, Ki, Kd são um PONTO DE PARTIDA. Precisarão de ajuste fino!
double Kp = 0.5, Ki = 0.8, Kd = 0; 
// Cria uma instância do PID
PID myPID(&current_rpm, &pwm_output, &setpoint_rpm, Kp, Ki, Kd, DIRECT);

// --- Configuração do PWM ---
#define PWM_TOP 511

// --- Função de Interrupção (ISR) ---
// Esta função é chamada AUTOMATICAMENTE toda vez que um pulso é detectado no pino RPM_PIN
void countPulse() {
  pulse_count++;
}

void setup() {
  // **1) Inicializa a Serial para o Modbus RTU**  
  Serial.begin(9600);  
  // --- Modbus ---
  modbus_configure(9600, 1, 0, TOTAL_REGS_SIZE, 0);
  holdingRegs[0] = 0; // Setpoint de RPM inicial
  holdingRegs[1] = 0; // RPM atual inicial

  // --- Pinos ---
  pinMode(PWM_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT);
  // Configura o pino de RPM com um resistor de pull-up interno, comum para sinais de tacho
  pinMode(RPM_PIN, INPUT_PULLUP); 

  // --- PWM (Timer1) ---
  TCCR1A = (1 << COM1A1) | (1 << WGM11);
  TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS10);
  ICR1 = PWM_TOP;
  OCR1A = 0; // Inicia com potência zero

  // --- Interrupção para RPM ---
  // Configura a interrupção para chamar a função 'countPulse' a cada borda de SUBIDA (RISING) do sinal
  attachInterrupt(digitalPinToInterrupt(RPM_PIN), countPulse, RISING);

  // --- PID ---
  setpoint_rpm = 0;
  myPID.SetMode(AUTOMATIC); // Liga o PID
  myPID.SetOutputLimits(0, PWM_TOP); // Limita a saída do PID para a faixa do nosso PWM (0-511)
  myPID.SetSampleTime(1000); // Define que o PID calculará a cada 1000 ms (1 seg)
}

void loop() {
  // Responde aos pedidos Modbus (leitura/escrita de Setpoint pelo SCADA)
  modbus_update(holdingRegs);

  // --- Lógica de Controle do Setpoint ---
  // O setpoint pode ser alterado tanto pelo SCADA quanto pelo controle físico
  setpoint_rpm = holdingRegs[0]; // Pega o setpoint que está no Modbus

  // Se o botão for pressionado, o potenciômetro define o novo setpoint
  if (digitalRead(BUTTON_PIN) == HIGH) {
    // Mapeia o potenciômetro para uma faixa de RPM (ex: 0 a 4000 RPM)
    // Ajuste o '4000' para o RPM máximo do seu cooler
    setpoint_rpm = map(analogRead(POT_PIN), 0, 1023, 0, 4000);
    holdingRegs[0] = setpoint_rpm; // Atualiza o registrador para o SCADA ver
    delay(200);
  }

  // --- Cálculo de RPM e Execução do PID ---
  // Executa o cálculo a cada 1 segundo (1000 ms) para ter uma leitura estável
  if (millis() - last_rpm_read_time >= 1000) {
    last_rpm_read_time = millis();

    // Desliga interrupções temporariamente para ler a variável 'pulse_count' com segurança
    noInterrupts();
    // A maioria dos fans dá 2 pulsos por revolução. RPM = (Pulsos/2) * 60
    current_rpm = (pulse_count * 30.0); 
    pulse_count = 0; // Zera o contador para a próxima medição
    interrupts(); // Liga as interrupções novamente

    // Atualiza o registrador Modbus com o RPM atual para o SCADA ler
    holdingRegs[1] = (unsigned int) current_rpm;  

    // Roda o algoritmo PID. Ele calculará o 'pwm_output' necessário.
    myPID.Compute(); 
  
    // Aplica a saída calculada pelo PID ao PWM do cooler
    OCR1A = pwm_output;
  }
}