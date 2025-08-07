# APC_Project_2025
Proggetto realizzato da Fienga Luigi [M63001733], Andrea Roscigno [M63001778] e Serena Savarese [M63001855]

# Automatic Water Dispenser

Il seguente lavoro è stato realizzato durante l’esame di “Architettura e Progetto dei Calcolatori”, nel quale è stato possibile lavorare con la scheda STM32F303VC per realizzare un dispenser automatico d’acqua, oltre alla scheda sono stati utilizzati: due sensori di prossimità, un display OLED, un servomotore e una seconda STM32F303VC (usata come ricevitore di dati da far visualizzare sullo schermo OLED). 

- La cartella `Nodo_Tx` contiene il codice della parte che rileva la presenza di un oggetto per attivare il servomotore per aprire il rubinetto dell'acqua e il sensore per valutare l'altezza del livello dell'acqua, quest'ultimo dato viene mandato al secondo schedino che fa da ricevitore
- La cartella `Nodo_Rx` contiene il codice della gestione dei dati ricevuti dal trasmettitore e la corretta visuallizzazione sul display del livello dell'acqua su tre soglie.

- [link](https://youtu.be/v_JCXkIU3Ts) per visualizzare il `video` dimostrativo del progetto.
- [link](https://drive.google.com/file/d/18TLXvo9eXZA-Rh1_AkP6cX_L0d7Ztzi6/view?usp=sharing) per visualizzare la `presentazione` del progetto.
