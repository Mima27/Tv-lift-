# Progetto Sollevamento TV con Mega2560 WiFi R3

Questo progetto controlla un sistema di sollevamento TV utilizzando Mega2560 WiFi R3 con ESP8266 integrato. Include controllo di 6 relè per pistoni e sportelli, lettura di sensori per altezza e corrente della TV, una pagina web per il controllo manuale e configurazione tempi, e pubblicazione dei dati via MQTT per integrazione con domotica.

## Componenti Hardware Necessari

- Mega2560 WiFi R3 con ESP8266 integrato
- Scheda relè a 6 canali
- 3 Pistoni lineari (attuatori) per sollevamento TV
- 2 Pistoni per sportelli (anteriore e posteriore)
- Sensore di altezza VL53L1X o VL53L0X su I2C
- Sensore di corrente ACS712
- Alimentazione appropriata

## Collegamenti

- Relè 1 (D2): Apertura sportello anteriore
- Relè 2 (D3): Chiusura pistone anteriore
- Relè 3 (D4): Alzata TV
- Relè 4 (D5): Abbassamento TV
- Relè 5 (D6): Apertura sportello posteriore
- Relè 6 (D7): Chiusura sportello posteriore
- Sensore altezza VL53: SDA su pin 20, SCL su pin 21, indirizzo I2C 0x29
- Sensore corrente ACS712: A0

## Logica di Funzionamento

### Apertura (quando TV accesa e altezza minima)
1. Attiva relè 1 per T1 ms (apertura anteriore)
2. Attiva relè 5 per T5 ms (apertura posteriore)
3. Attiva relè 3 fino a altezza massima (alzata TV)
4. Attiva relè 2 per T2 ms (chiusura anteriore)
5. Sportello posteriore rimane aperto

### Chiusura (quando TV spenta)
1. Attiva relè 1 per T1 ms (apertura anteriore)
2. Attiva relè 4 fino a altezza minima (abbassamento TV)
3. Attiva relè 2 per T2 ms (chiusura anteriore)
4. Attiva relè 6 per T6 ms (chiusura posteriore)

## Configurazione Software

1. Installa le board Arduino AVR e ESP8266 in Arduino IDE
2. Installa librerie: Adafruit_VL53L1X oppure Adafruit_VL53L0X, ESP8266WiFi, ESP8266WebServer, PubSubClient
3. Modifica configurazioni WiFi e MQTT
4. Carica mega2560_logic sul Mega e mega2560_esp sull'ESP8266 integrato

## Controllo

- **Web**: Accedi a http://IP_ARDUINO per controlli e impostazioni tempi
- **MQTT**: Comandi "open"/"close"/"stop" su topic command, stato su topic status
- **Automatico**: Basato su stato TV e altezza
- **Diagnostica sensore**: la UI mostra stato sensore, modello e indirizzo I2C rilevato

## Tempi Configurabili

Tutti i tempi sono salvati in EEPROM e configurabili via web:
- T1: Apertura anteriore
- T2: Chiusura anteriore
- T3: Alzata TV (non usato direttamente, basato su sensore)
- T4: Abbassamento TV (non usato direttamente)
- T5: Apertura posteriore
- T6: Chiusura posteriore

## Sicurezza

- Stop automatico alle soglie altezza
- Controllo stato TV per evitare movimenti errati
- Possibilità di stop manuale
