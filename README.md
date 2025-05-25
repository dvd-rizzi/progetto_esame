__________             ._____________       .__    .___      
\______   \_____     __| _/\______   \ ____ |__| __| _/______
 |    |  _/\__  \   / __ |  |    |  _//  _ \|  |/ __ |/  ___/
 |    |   \ / __ \_/ /_/ |  |    |   (  <_> )  / /_/ |\___ \ 
 |______  /(____  /\____ |  |______  /\____/|__\____ /____  >
        \/      \/      \/         \/               \/    \/ 

##Introduzione

Il nostro progetto consiste nell'implementazione di una simulazione del comportamento di uno stormo di uccelli in volo in uno spazio bidimensionale.

Abbiamo utilizzato tre "regole":
1. La **regola** di **separazione** evita che i *boids* collidano tra loro;
2. La **regola** di **allineamento** fa in modo che i *boids* procedano nella stessa direzione dello stormo;
3. La **regola** di **coesione** induce un *boid* a volare verso il centro di massa dei *boids* vicini.