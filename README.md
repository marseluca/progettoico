# Progetto ICO
## Variabili
<ul>
<li>robotSize -> diametro del robot, in metri</li>
<li>delta_s -> safety margin, nell'ottimizzazione, i robot dovranno rispettare questa distanza minima tra loro</li>
<li>collisionThreshold -> non importante per ora</li>
<li>maxVelocity -> velocità massima dei robot (usato per il trajectory planning)</li>
<li>maxAcceleration -> accelerazione massima dei robot (usato per il trajectory planning)</li>
</ul>

## Opzioni
<ul>
<li>animation -> se settato sue true, ti permette di generare il video dell'animazione. Quando termina l'animazione, il video viene salvato come "warehouse.mp4" nella cartella dello script</li>
<li>animVelocity -> serve solo se animation = true, velocità dell'animazione (2 = velocità 2x etc.)</li>
<li>recordAnimation -> da lasciare sempre su true</li>
<li>solveCollisions, preloadOptimization -> per ora non servono</li>
<li>plotVelocities -> scegli se plottare profili di posizione, velocità e accelerazione</li>
<li>plotCollisions -> scegli se plottare le collisioni</li>
</ul>

## Funzioni importanti
<ul>
  <li>interpolatePath2(path,0,0) -> Dato il singolo path in ingresso come insieme di punti, restituisce la traiettoria come variabile "trajectories" di tipo cell, in cui la cell j-esima contiene la traiettoria del robot j-esimo come variabile struct. Mentre il path contiene solo l'informazione geometrica sul percorso del robot (sequenza di punti come coordinate 2d) la traiettoria interpola il set di punti del path (ossia genera un set di punti intermedi in modo smooth) e assegna una sequenza temporale al set di punti. Gli ultimi due argomenti ("0,0") non servono al momento e devono essere lasciati così.</li>
  <li>pp_checkCollisionForOneRobot(paths,trajectories,collisionThreshold,j) -> Dato il robot j, restituisce tutte le collisioni che subisce il robot durante tutta la simulazione. Una collisione avviene quando il robot j si trova a una distanza minore di collisionThreshold da un qualsiasi altro robot. Le collisioni vengono restituite in una variabile "collisions" di tipo cell. La cell j-esima contiene l'elenco delle collisioni con il robot j-esimo come array</li>
</ul>

## Per iniziare
Per familiarizzare col codice si può innanzitutto esaminare il contenuto della variabile "trajectories".<br>
Successivamente si può provare a cambiare leggermente il set di punti dei path per vedere come cambiano le traiettorie che vengono generate.<br>
Successivamente si possono iniziare a settare una alla volta le variabili "animation", "plotVelocities" e "plotCollisions" su true per vedere cosa succede.<br>

[Presentazione Progetto ICO.pptx](https://github.com/user-attachments/files/18867524/Presentazione.Progetto.ICO.pptx)

## Per il futuro
L'obiettivo sarà quello di creare un algoritmo di ottimizzazione che generi delle traiettorie per le quali non ci siano collisioni.
Il problema di ottimizzazione può essere impostato così:<br>

![image](https://github.com/user-attachments/assets/d3734f2c-a98f-459c-940a-28da24ab6755)

<br>
Il modo in cui si costruisce l'algoritmo dipende dal solver di matlab che viene usato. Ma nel mio caso col Genetic Algorithm l'avevo impostato così (in modo semplificato):<br>
![image](https://github.com/user-attachments/assets/2970322d-7c93-4e73-8d8d-af560ca084c0)<br>
![image](https://github.com/user-attachments/assets/9e84bc20-f89b-47bf-804e-dbb2fee3304b)



