# Quatron-V0 — Modelagem Cinemática e Controle de Estabilidade de Robô Quadrúpede

> **Projeto de Iniciação Científica — UTFPR (Campus Apucarana)**  
> Repositório oficial referente ao trabalho *"Desenvolvimento de Modelagem e Controle de Robô Quadrúpede Utilizando Impressão 3D"*, apresentado no **XXXI Seminário de Iniciação Científica e Tecnológica (SICITE 2026)**.

Sistema completo de modelagem cinemática, planejamento de marcha (*crawl gait*), compensação postural em malha aberta orientada pelo Centro de Massa (CoM) e atuação eletromecânica para o robô quadrúpede autoral **Quatron-V0** (12 graus de liberdade).

---

## Objetivo e Abordagem

Diferente de estratégias rígidas com trajetórias fixas, este projeto implementa uma **compensação cinemática baseada na projeção 2D do Centro de Massa (CoM)**. Durante a fase de elevação de cada perna (*swing*), o algoritmo reajusta proativamente a postura do chassi em direção ao centroide do triângulo formado pelas patas apoiadas, garantindo a estabilidade estática sem depender de modelos dinâmicos complexos ou sensores de força nas extremidades.

---

## Arquitetura do Sistema

```text
+-------------------------------------------------------+
|                   PC (MATLAB)                         |
|  • Modelagem RigidBodyTree (Peter Corke Toolbox)      |
|  • Planejamento de Trajetória (mstraj) & IK Numérica  |
|  • Compensação do Centro de Massa (Centroide)         |
|  • Filtro Cosseno (Curva em S) & Análise de MSM       |
+---------------------------+---------------------------+
                            |
             USB Serial (115200 bps / 50 Hz)
              Pacote CSV: "q1,q2,...,q12\n"
                            |
                            v
+-------------------------------------------------------+
|               Arduino Mega 2560                       |
|  • Alimentado via USB 5V do computador                |
|  • Parsing dos 12 ângulos e espelhamento mecânico     |
+---------------------------+---------------------------+
                            |
                     Barramento I2C
                       (SDA / SCL)
                            |
                            v
+-------------------------------------------------------+
|         Driver PWM Adafruit PCA9685 (50 Hz)           |
|  • Alimentado por Fonte de Bancada Externa (6V)       |
|  • Distribuição de corrente para os 12 servo motores  |
+---------------------------+---------------------------+
                            |
                 12 Canais PWM (0 a 11)
                            |
                            v
+-------------------------------------------------------+
|       Protótipo Físico Quatron-V0 (Impresso em 3D)    |
|  • 4 Pernas x 3 GDL (Ombro Roll, Coxa, Canela)        |
+-------------------------------------------------------+
```

---

## Especificações Mecânicas e de Hardware

### Parâmetros Dimensionais do Protótipo (Quatron-V0)
* **Fabricação:** Estrutura autoral impressa em 3D com massa total de **$1,8\text{ kg}$**.
* **Base Central:** Retangular de **$300,00\text{ mm} \times 180,00\text{ mm}$** (entre-eixos de **$135,20\text{ mm}$** e envergadura total do quadril de **$287,20\text{ mm}$**).
* **Cinemática das Pernas (3 GDL por perna):**
  * Offset de suporte: **$43,46\text{ mm}$** | Offset vertical do chassi ao eixo da coxa: **$60,00\text{ mm}$**.
  * Comprimento da coxa (fêmur): **$147,60\text{ mm}$**.
  * Comprimento da canela (tíbia): **$127,60\text{ mm}$** (alcance estendido total de **$275,20\text{ mm}$**).
  * Largura da base do pé: **$50,00\text{ mm}$** | Altura estrutural montada: **$335,20\text{ mm}$**.

### Eletrônica e Alimentação

| Componente | Especificação / Função |
| :--- | :--- |
| **Microcontrolador** | **Arduino Mega 2560** (programado via Arduino IDE) |
| **Driver de Servos** | **Adafruit PCA9685** (16 canais, 12 bits, I2C @ $50\text{ Hz}$) |
| **Atuadores** | **12x Servomotores MG996R** (torque nominal de **$11\text{ kg}\cdot\text{cm}$** a $6\text{ V}$) |
| **Fonte Lógica** | Porta **USB $5\text{ V}$** do computador (alimenta exclusivamente o Arduino Mega) |
| **Fonte de Potência** | **Fonte de Bancada ajustada em $6\text{ V}$** conectada ao barramento `V+` do PCA9685 |

> **Nota sobre Alimentação:** Como os 12 servomotores MG996R demandam corrente de pico muito superior à capacidade do regulador interno do Arduino Mega, a potência do **PCA9685** é isolada e suprida pela **fonte externa de $6\text{ V}$** (mantendo o **GND comum** entre Arduino, PCA9685 e fonte).

---

## Parâmetros de Controle e Simulação

* **Marcha Estática (*Crawl Gait*):** Ciclo sequencial com **75% em fase de apoio** ($z_d = -290,00\text{ mm}$) e **25% em fase de balanço** ($z_u = -250,00\text{ mm}$), com defasamento de $1/4$ de ciclo entre as pernas.
* **Compensação do CoM:** Deslocamento proporcional em direção ao centroide do triângulo de suporte com **ganho de $0,6$** e suavização por **filtro passa-baixa de coeficiente $0,1$**.
* **Filtro de Transição em Curva "S":** Interpolação cossenoidal ($T = 20$ quadros a $50\text{ Hz}$) acionada sempre que a variação angular excede **$0,5\text{ rad}$**, evitando trancos mecânicos.
* **Critério de Estabilidade (MSM):** Cálculo geométrico da menor distância ortogonal entre a projeção do CoM e as arestas do polígono de suporte orientado no sentido anti-horário, adotando **$30\text{ mm}$** como limiar mínimo de segurança (Park et al., 2006).

---

## Resultados Obtidos e Análise *Sim-to-Real*

### 1. Validação em Ambiente Simulado (MATLAB @ $50\text{ Hz}$)
| Cenário de Teste | Margem Mínima de Estabilidade (MSM) | Comportamento Observado |
| :--- | :---: | :--- |
| **Sem Controle Postural** | $< 0\text{ mm}$ | Projeção do CoM sai da área de suporte durante a elevação da pata, resultando em tombamento. |
| **Com Controle de Postura (CoM)** | **$63,302\text{ mm}$ a $82,76\text{ mm}$** | Execução contínua e equilibrada; MSM permaneceu mais que o dobro acima do limiar de segurança de $30\text{ mm}$. |

### 2. Ensaios Experimentais e Gargalo de Hardware (*Sim-to-Real Gap*)
Nos testes físicos com o protótipo impresso em 3D, a comunicação serial entre o MATLAB e o Arduino Mega operou de forma síncrona a **$50\text{ Hz}$**. Entretanto, observou-se **saturação mecânica instantânea dos atuadores** ao tentar sustentar o robô sobre o solo:
* **Torque Exigido pelo Protótipo:** Considerando a massa total de **$1,8\text{ kg}$** e o braço de alavanca operacional das pernas ($147,6\text{ mm}$ a $220,0\text{ mm}$), cada junta exige entre **$13,3\text{ kg}\cdot\text{cm}$ e $19,8\text{ kg}\cdot\text{cm}$** de torque contínuo.
* **Limite dos Atuadores:** Os servomotores **MG996R** possuem torque máximo de travamento (*stall torque*) de **$11\text{ kg}\cdot\text{cm}$** (a $6\text{ V}$), tornando-se insuficientes para vencer o peso próprio da estrutura física atual.

---

## Trabalhos Futuros (Roadmap)

- [ ] **Otimização Estrutural:** Redução de massa do chassi e dos elos impressos em 3D via otimização topológica.
- [ ] **Upgrade de Atuação:** Substituição dos atuadores MG996R por servomotores de alto torque ($> 20\text{ kg}\cdot\text{cm}$) com engrenagens metálicas.
- [ ] **Controle em Malha Fechada:** Integração de uma Unidade de Medição Inercial (**IMU**) para realimentação da inclinação do tronco em terrenos irregulares.

---

## Como Executar

1. **Hardware:** Conecte o Arduino Mega via USB ($5\text{ V}$) ao PC e alimente o módulo PCA9685 com a fonte de bancada ajustada em **$6\text{ V}$** (com GND compartilhado).
2. **Firmware (Arduino IDE):** Instale a biblioteca `Adafruit PWM Servo Driver Library`, abra `arduino/quatron_servo_driver/quatron_servo_driver.ino` e faça o upload para o **Arduino Mega 2560**. Os 12 servos serão inicializados na posição neutra ($90^\circ$).
3. **Controle (MATLAB):** Certifique-se de ter o *Robotics System Toolbox* / *Robotics Toolbox (Peter Corke)* instalado, abra `matlab/quatron_kinematics_control.m` e execute as seções para gerar a marcha, plotar a curva de MSM, rodar a simulação 3D ou transmitir os ângulos via serial.

---

## Estrutura do Repositório

```text
├── arduino/
│   └── quatron_servo_driver/
│       └── quatron_servo_driver.ino   # Firmware de recepção serial e acionamento PWM (PCA9685)
├── matlab/
│   └── quatron_kinematics_control.m   # Modelagem RBT, marcha, controle de CoM e análise de MSM
└── README.md                          # Documentação técnica e científica do projeto
```

---

## Autores, Agradecimentos e Referências

* **Autores:** João Pedro de Oliveira Mendes Sereno & Prof. Mauricio Eiji Nakai — *Universidade Tecnológica Federal do Paraná (UTFPR), Campus Apucarana*.
* **Apoio Institucional:** Agradecimentos à **UTFPR**, ao **Laboratório de Fabricação Multiusuário (LARP)**, ao **CNPq** e à **Fundação Araucária** pelo suporte técnico e fomento à pesquisa.

### Referências Principais
1. **CORKE, P.** *Robotics, Vision and Control: Fundamental Algorithms in MATLAB*. Springer Science & Business Media, v. 73, 2017.
2. **PARK, I. W. et al.** *Inverse kinematics analysis and COG trajectory planning algorithms for stable walking of a quadruped robot with redundant DOFs*. IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), p. 3844–3849, 2006.
3. **PEDRO, G. D. G.; FREITAS, G. M.** *Controle dinâmico de marcha e passo para um robô quadrúpede utilizando curvas de Bézier*. Congresso Brasileiro de Automática (CBA), v. 3, p. 1–8, 2022.
4. **BARASUOL, V.** *Um sistema de controle reativo para locomoção de robôs quadrúpedes*. Tese de Doutorado, Universidade Federal de Santa Catarina (UFSC), 2013.
5. **ZHANG, S.; ZHANG, X.; ZHANG, G.** *Gait control and stability analysis for quadruped robots with multi-input multi-output nonlinear dynamics*. IEEE Access, v. 8, p. 12345–12356, 2020.