#include "BluetoothA2DPSink.h"
#include "driver/ledc.h"

// Configuração dos pinos dos motores de vibração
#define MOTOR_1_PIN 25
#define MOTOR_2_PIN 26
#define MOTOR_3_PIN 27
#define MOTOR_4_PIN 32

// Configurações de processamento
#define SAMPLE_BUFFER_SIZE 512
#define ENERGY_HISTORY_SIZE 10

// Estrutura para análise de áudio
typedef struct {
    float energy_history[ENERGY_HISTORY_SIZE];
    int history_index;
    unsigned long last_beat_time;
    float current_energy;
    float average_energy;
    bool beat_detected;
} AudioProcessor;

typedef struct {
    int motor1;
    int motor2;
    int motor3;
    int motor4;
    bool beat_active;
} MotorState;

BluetoothA2DPSink a2dp_sink;
AudioProcessor audio_processor = {0};
MotorState motor_state = {0, 0, 0, 0, false};

// Configuração PWM
void setupPWM() {
    ledcSetup(0, 1000, 8);
    ledcSetup(1, 1000, 8);
    ledcSetup(2, 1000, 8);
    ledcSetup(3, 1000, 8);
    
    ledcAttachPin(MOTOR_1_PIN, 0);
    ledcAttachPin(MOTOR_2_PIN, 1);
    ledcAttachPin(MOTOR_3_PIN, 2);
    ledcAttachPin(MOTOR_4_PIN, 3);
    
    // Inicia com todos os motores desligados
    stopAllMotors();
}

void stopAllMotors() {
    ledcWrite(0, 0);
    ledcWrite(1, 0);
    ledcWrite(2, 0);
    ledcWrite(3, 0);
    
    motor_state.motor1 = 0;
    motor_state.motor2 = 0;
    motor_state.motor3 = 0;
    motor_state.motor4 = 0;
}

// Função simples para calcular energia do áudio
float calculateAudioEnergy(int16_t *samples, int sample_count) {
    long sum = 0;
    // Analisa uma amostragem menor para performance
    int step = sample_count / 64;

    // printf("sample %d\n", sample_count);
    // printf("step %d\n", step); // Debug STEP

    if (step < 1) step = 1;
    
    for (int i = 0; i < sample_count; i += step) {
        sum += abs(samples[i]);
        printf("samples[%d] = %d|", i, samples[i]);
    }
    printf("sum = %d", 
    float energy = (float)sum / (sample_count / step) / 32768.0f;
    return energy;
}

// Detecta se há batimento baseado na energia
bool detectBeat(float instant_energy) {
    unsigned long current_time = millis();
    
    // Atualiza histórico
    audio_processor.energy_history[audio_processor.history_index] = instant_energy;
    audio_processor.history_index = (audio_processor.history_index + 1) % ENERGY_HISTORY_SIZE;
    
    // Calcula energia média
    float sum = 0;
    for (int i = 0; i < ENERGY_HISTORY_SIZE; i++) {
        sum += audio_processor.energy_history[i];
    }
    audio_processor.average_energy = sum / ENERGY_HISTORY_SIZE;
    
    // Detecta beat simples
    if (instant_energy > audio_processor.average_energy * 1.5f && 
        instant_energy > 0.05f &&
        current_time - audio_processor.last_beat_time > 100) {
        audio_processor.last_beat_time = current_time;
        return true;
    }
    
    return false;
}

// Função de callback para processamento de áudio
void audio_data_callback(const uint8_t *data, uint32_t length) {
    int16_t *samples = (int16_t*)data;
    int sample_count = length / 2;
    
    // Calcula energia instantânea
    float instant_energy = calculateAudioEnergy(samples, sample_count);
    audio_processor.current_energy = instant_energy;
    
    // Detecta batimento
    bool has_beat = detectBeat(instant_energy);
    
    // Processa vibração baseado na energia e batimentos
    processVibration(instant_energy, has_beat);
}

// Processa a vibração dos motores
void processVibration(float energy, bool has_beat) {
    // Mapeia energia para intensidade base (sempre presente)
    int base_intensity = mapToBaseIntensity(energy);
    
    if (has_beat) {
        // Modo batimento - intensidade alta
        int beat_intensity = mapToBeatIntensity(energy);
        activateBeatVibration(beat_intensity);
        motor_state.beat_active = true;
    } else {
        // Modo contínuo - intensidade média/baixa
        activateContinuousVibration(base_intensity);
        motor_state.beat_active = false;
    }
}

// Mapeia energia para intensidade base (contínua)
int mapToBaseIntensity(float energy) {
    // Garante que sempre há alguma vibração quando há áudio
    float intensity = energy * 3.0f;
    intensity = constrain(intensity, 0.0f, 1.0f);
    
    // Range: 40-120 para vibração contínua suave
    return (int)(intensity * 80 + 40);
}

// Mapeia energia para intensidade de batimento
int mapToBeatIntensity(float energy) {
    float intensity = energy * 5.0f;
    intensity = constrain(intensity, 0.0f, 1.0f);
    
    // Range: 150-255 para batimentos fortes
    return (int)(intensity * 105 + 150);
}

// Ativa vibração contínua
void activateContinuousVibration(int intensity) {
    // Distribui a vibração entre os motores baseado em padrões simples
    motor_state.motor1 = intensity;                    // Motor 1: sempre ativo
    motor_state.motor2 = intensity * 0.8;             // Motor 2: um pouco menos
    motor_state.motor3 = intensity * 0.6;             // Motor 3: menos ainda
    motor_state.motor4 = intensity * 0.7;             // Motor 4: médio
    
    applyMotorVibration();
}

// Ativa vibração de batimento
void activateBeatVibration(int intensity) {
    // Para batimentos, ativa todos os motores com força
    motor_state.motor1 = intensity;
    motor_state.motor2 = intensity;
    motor_state.motor3 = intensity;
    motor_state.motor4 = intensity;
    
    applyMotorVibration();
}

// Aplica as vibrações aos motores
void applyMotorVibration() {
    ledcWrite(0, motor_state.motor1);
    ledcWrite(1, motor_state.motor2);
    ledcWrite(2, motor_state.motor3);
    ledcWrite(3, motor_state.motor4);
}

// Configuração do Bluetooth A2DP
void setupA2DP() {
    static i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
        .sample_rate = 44100,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
        .communication_format = I2S_COMM_FORMAT_I2S_MSB,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 8,
        .dma_buf_len = 256,
        .use_apll = false,
        .tx_desc_auto_clear = true,
        .fixed_mclk = 0
    };
    
    a2dp_sink.set_i2s_config(i2s_config);
    a2dp_sink.set_stream_reader(audio_data_callback, false);
    a2dp_sink.start("ESP32-MusicVibe");
    a2dp_sink.set_volume(100);
}

void setup() {
    Serial.begin(115200);
    
    // Configura pinos dos motores
    pinMode(MOTOR_1_PIN, OUTPUT);
    pinMode(MOTOR_2_PIN, OUTPUT);
    pinMode(MOTOR_3_PIN, OUTPUT);
    pinMode(MOTOR_4_PIN, OUTPUT);
    
    // Inicializa PWM
    setupPWM();
    
    Serial.println("SISTEMA DE VIBRAÇÃO MUSICAL SIMPLIFICADO");
    Serial.println("Modo: Vibração Contínua + Batimentos");
    delay(1000);
    
    setupA2DP();
    
    Serial.println("Sistema pronto - Conecte via Bluetooth: ESP32-MusicVibe");
    Serial.println("Aguardando áudio...");
}

void loop() {
    // Debug a cada segundo
    static unsigned long last_debug = 0;
    if (millis() - last_debug > 1000) {
        Serial.printf("Energia: %.3f | Beat: %s | Motores: %d,%d,%d,%d\n",
                     audio_processor.current_energy,
                     motor_state.beat_active ? "SIM" : "não",
                     motor_state.motor1,
                     motor_state.motor2,
                     motor_state.motor3,
                     motor_state.motor4);
        last_debug = millis();
    }
    
    // Pequeno delay para não sobrecarregar
    delay(50);
}