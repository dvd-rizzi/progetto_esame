```_________             ._____________       .__    .___      
\______   \_____     __| _/\______   \ ____ |__| __| _/______
 |    |  _/\__  \   / __ |  |    |  _//  _ \|  |/ __ |/  ___/
 |    |   \ / __ \_/ /_/ |  |    |   (  <_> )  / /_/ |\___ \ 
 |______  /(____  /\____ |  |______  /\____/|__\____ /____  >
        \/      \/      \/         \/               \/    \/ 
```
# Introduzione

Il nostro progetto consiste nell'implementazione di un programma che simula il comportamento di uno stormo di uccelli in volo in uno spazio bidimensionale.

Abbiamo utilizzato tre "regole":
1. La **regola** di **separazione** evita che i *boids* collidano tra loro;
2. La **regola** di **allineamento** fa in modo che i *boids* procedano nella stessa direzione dello stormo;
3. La **regola** di **coesione** induce un *boid* a volare verso il centro di massa dei *boids* vicini.

Per compilare il programma sono necessari i seguenti comandi:
```shell
cmake -S . -B build -G"Ninja Multi-Config"
cmake --build build --config Debug
cmake --build build --config Debug --target test
cmake --build build --config Release
cmake --build build --config Release --target test
```
e per eseguire utilizzare:
```shell
./build/Release/boids
./build/Debug/boids
```
All'avvio del programma sarà necessario inserire dei parametri per la formazione dello stormo:
1. Il **numero di Boids (N_)**;
2. La **distanza massima (d_)** entro cui le regole di coesione e di allineamento hanno effetto; 
3. La **distanza minima (ds_)** al di sotto della quale entra in gioco la regola di separazione;
4. Il **fattore di separazione (s_)** che determina l'intensità di repulsione dei boids vicini;
5. Il **fattore di allineamento (a_)** che detertmina la tendenza dei boids ad uniformare il verso delle proprie velocità (*è necessario che tale fattore sia strettamente minore di 1*);
6. Il **fattore di coesione (c_)** che determina l'intensità dell'attrazione del boid verso il centro di massa dei boids limitrofi.

A seconda della scelta dei parametri, la simulazione potrà comportarsi in modo differente, rendendo più o meno apprezzabile la formazione dello stormo. Per un'inizializzazione ottimale si consiglia di scegliere i parametri nei seguenti range:
1. 10 < **N_** < 100
2. 10 < **d_** < 15>
3. 2 < **ds_** < 4
4. 2 < **s_** < 5
5. 0.2 < **a_** 0.5
6. 0.05 < **c_** < 0.1



