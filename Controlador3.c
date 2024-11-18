#include <webots/robot.h>
#include <webots/supervisor.h>
#include <webots/distance_sensor.h>
#include <webots/motor.h>
#include <webots/led.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>

// Constantes de configuração
#define CICLO_BASE 32
#define VELOCIDADE_MAXIMA 6.28
#define TEMPO_ESPERA 64.0
#define NUM_SENSORES 8
#define NUM_LEDS 10
#define TEMPO_LOC_ROBO 160
#define TEMPO_RE 160
#define LIMIAR 100
#define LIMIAR_CICLOS 10
#define TOLERANCIA_POSICAO 5

typedef enum Estados {
  ROBO_INICIALIZANDO = 0,
  ROBO_AGUARDANDO,
  ROBO_PROCURANDO_CAIXA,
  ROBO_DANDO_RE,
  ROBO_ENCONTROU_CAIXA
} Estados;

// Estrutura para armazenar os dispositivos do robô
typedef struct {
  WbDeviceTag roda_esq;
  WbDeviceTag roda_dir;
  WbDeviceTag sensores[NUM_SENSORES];
  WbDeviceTag leds[NUM_LEDS];

  int pos_x[2];
  int pos_y[2];

  Estados estado;

  unsigned int passos_re;
  unsigned int passos_loc;

  double velocidade_esq;
  double velocidade_dir;
} DispositivosRobo;

DispositivosRobo robo;
WbNodeRef objeto_alvo;
WbNodeRef objeto_robo;

double pos_inicial[3];   // Posição inicial da caixa
const double *pos_atual; // Posição atual da caixa
const double *pos_robo;  // Posição atual do robô

// Variáveis para suavização
static int contador_obstaculo_esq = 0;
static int contador_obstaculo_dir = 0;

// Função para configurar os dispositivos
DispositivosRobo configurar_dispositivos() {
  DispositivosRobo dispositivos;

  // Configuração das rodas
  dispositivos.roda_esq = wb_robot_get_device("left wheel motor");
  dispositivos.roda_dir = wb_robot_get_device("right wheel motor");

  wb_motor_set_position(dispositivos.roda_esq, INFINITY);
  wb_motor_set_position(dispositivos.roda_dir, INFINITY);

  wb_motor_set_velocity(dispositivos.roda_esq, 0.0);
  wb_motor_set_velocity(dispositivos.roda_dir, 0.0);

  // Configuração dos sensores
  for (int i = 0; i < NUM_SENSORES; i++) {
    char nome_sensor[8];
    sprintf(nome_sensor, "ps%d", i);
    dispositivos.sensores[i] = wb_robot_get_device(nome_sensor);
    wb_distance_sensor_enable(dispositivos.sensores[i], CICLO_BASE);
  }

  // Configuração dos LEDs
  for (int i = 0; i < NUM_LEDS; i++) {
    char nome_led[8];
    sprintf(nome_led, "led%d", i);
    dispositivos.leds[i] = wb_robot_get_device(nome_led);
    wb_led_set(dispositivos.leds[i], 0);
  }

  return dispositivos;
}

// Função para suavizar detecção de obstáculos
bool verificar_suavizacao(int *contador, bool obstaculo_detectado) {
  if (obstaculo_detectado) {
    (*contador)++;
    if (*contador >= LIMIAR_CICLOS) {
      *contador = LIMIAR_CICLOS; // Limitar o contador
      return true;
    }
  } else {
    (*contador)--;
    if (*contador <= 0) {
      *contador = 0; // Limitar o contador
    }
  }
  return false;
}

// Função para verificar sensores
void verificar_sensores() {
  bool sensor_dir_ativo = false;
  bool sensor_esq_ativo = false;

  // Verificar sensores do lado direito
  for (int i = 0; i < 4; i++) {
    if (wb_distance_sensor_get_value(robo.sensores[i]) > LIMIAR) {
      sensor_dir_ativo = true;
      break;
    }
  }

  // Verificar sensores do lado esquerdo
  for (int i = 4; i < NUM_SENSORES; i++) {
    if (wb_distance_sensor_get_value(robo.sensores[i]) > LIMIAR) {
      sensor_esq_ativo = true;
      break;
    }
  }

  // Suavização para evitar mudanças rápidas
  bool obstaculo_esq = verificar_suavizacao(&contador_obstaculo_esq, sensor_esq_ativo);
  bool obstaculo_dir = verificar_suavizacao(&contador_obstaculo_dir, sensor_dir_ativo);

  // Ajustar velocidades com prioridade
  if (obstaculo_esq && obstaculo_dir) {
    // Prioridade: girar no lugar para evitar colisões
    robo.velocidade_esq = -VELOCIDADE_MAXIMA;
    robo.velocidade_dir = VELOCIDADE_MAXIMA;
  } else if (obstaculo_esq) {
    // Obstáculo à esquerda
    robo.velocidade_esq = VELOCIDADE_MAXIMA;
    robo.velocidade_dir = 0.2 * VELOCIDADE_MAXIMA;
  } else if (obstaculo_dir) {
    // Obstáculo à direita
    robo.velocidade_esq = 0.2 * VELOCIDADE_MAXIMA;
    robo.velocidade_dir = VELOCIDADE_MAXIMA;
  } else {
    // Sem obstáculos
    robo.velocidade_esq = VELOCIDADE_MAXIMA;
    robo.velocidade_dir = VELOCIDADE_MAXIMA;
  }
}

// Função principal de controle
void controlar_robo() {
  while (wb_robot_step(CICLO_BASE) != -1) {
    WbNodeRef objeto_alvo = wb_supervisor_node_get_from_def("leve");
    WbNodeRef objeto_robo = wb_supervisor_node_get_from_def("Supervisor");

    const double *pos_atual = wb_supervisor_node_get_position(objeto_alvo);
    const double *pos_robo = wb_supervisor_node_get_position(objeto_robo);

    switch (robo.estado) {
      case ROBO_INICIALIZANDO:
        memset(pos_inicial, 0, 3 * sizeof(double));
        static double contador = 0.0;

        robo.passos_loc = 0;
        robo.passos_re = 0;

        robo = configurar_dispositivos();

        robo.estado = ROBO_AGUARDANDO;
        printf("Robo Inicializado\n");
        break;

      case ROBO_AGUARDANDO:
        contador++;
        if (contador == TEMPO_ESPERA) {
          memcpy(pos_inicial, pos_atual, 3 * sizeof(double));
          printf("Sistema iniciado - monitorando movimento\n");
          printf("Posição: %.4f, %.4f, %.4f\n", pos_inicial[0], pos_inicial[1], pos_inicial[2]);
          contador = 0;
          robo.estado = ROBO_PROCURANDO_CAIXA;
        }
        break;

      case ROBO_PROCURANDO_CAIXA:
      robo.velocidade_esq = VELOCIDADE_MAXIMA;
      robo.velocidade_dir = VELOCIDADE_MAXIMA;

      if (pos_inicial[0] != pos_atual[0] || pos_inicial[1] != pos_atual[1]) {
        printf("Movimento detectado em: %.9f, %.9f\n", pos_atual[0], pos_atual[1]);
        robo.estado = ROBO_ENCONTROU_CAIXA;
      }

      if (robo.passos_loc == 0) {
        robo.pos_x[0] = (int)(pos_robo[0] * 1000);
        robo.pos_y[0] = (int)(pos_robo[1] * 1000);
      }

      robo.passos_loc++;
      if (robo.passos_loc >= TEMPO_LOC_ROBO) {
        robo.pos_x[1] = (int)(pos_robo[0] * 1000);
        robo.pos_y[1] = (int)(pos_robo[1] * 1000);

        if (abs(robo.pos_x[0] - robo.pos_x[1]) < TOLERANCIA_POSICAO &&
            abs(robo.pos_y[0] - robo.pos_y[1]) < TOLERANCIA_POSICAO) {
          printf("Robo travado\n");
          robo.estado = ROBO_DANDO_RE;
        }

        robo.passos_loc = 0;
      }

      verificar_sensores();
        break;

      case ROBO_DANDO_RE:
        robo.velocidade_esq = -VELOCIDADE_MAXIMA;
        robo.velocidade_dir = -0.25 * VELOCIDADE_MAXIMA;
        robo.passos_re++;

        if (robo.passos_re >= TEMPO_RE) {
          robo.passos_re = 0;
          robo.estado = ROBO_PROCURANDO_CAIXA;
        }
        break;

      case ROBO_ENCONTROU_CAIXA:
        robo.velocidade_esq = 0.0;
        robo.velocidade_dir = 0.0;

        for (int i = 0; i < NUM_LEDS; i++) {
          wb_led_set(robo.leds[i], 1);
        }
        break;

      default:
        break;
    }

    wb_motor_set_velocity(robo.roda_esq, robo.velocidade_esq);
    wb_motor_set_velocity(robo.roda_dir, robo.velocidade_dir);
  }
}

int main() {
  wb_robot_init();
  controlar_robo();
  wb_robot_cleanup();
  return 0;
}
