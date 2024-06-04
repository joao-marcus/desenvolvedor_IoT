# Projeto Wokwi ESP32 MQTT e Controle de LED

## Instruções de Compilação e Teste

### 1. Acesso ao Projeto no Wokwi:
   - Abra o projeto na plataforma Wokwi.

### 2. Configuração do Hardware:
   - Monte o circuito com o ESP32, o LED NeoPixel, o sensor DHT22 e, opcionalmente, um botão no pino GPIO 13.

### 3. Configuração do Software:
   - Não são necessárias configurações adicionais no editor do Wokwi.

### 4. Simulação e Teste:
   - Inicie a simulação.
   - Verifique no console de saída se o ESP32 se conecta à rede Wi-Fi e ao broker MQTT.
   - Interaja com o simulador para testar o controle do LED NeoPixel e observar os relatórios de temperatura, umidade e status do LED.

### 5. Depuração e Ajustes:

#### Controlar o LED:
   - Você pode enviar comandos MQTT para o tópico `topic_led_control` para controlar a cor e o estado do LED NeoPixel.
   - O botão push pode ser usado para alternar entre diferentes estados do LED.

   Exemplo de Mensagem MQTT para Controlar o LED (cor;state):
```
    2;1

 ```
- color pode ser: 0 (VERDE), 1 (AMARELO), 2 (VERMELHO), 3 (AZUL)
- state pode ser: 0 (DESLIGADO), 1 (LIGADO), 2 (PISCANDO_1S), 3 (PISCANDO_0_3S)

## Observações

- Certifique-se de usar as versões mais recentes das bibliotecas.
- Verifique a conexão do hardware e revise o código em caso de problemas na conexão com a rede.
- Codigo foi feito no ESP32 pois no simulador ESP32 S3 o Led NeoPixel não ligava e não compilava as bibliotecas dos sensores corretamente utilizando ESP-IDF 