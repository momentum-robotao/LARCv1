# Regras LARC simulation 2024

Forum com atualizações de regras Larc/robocup junior, precisa olhar: https://junior.forum.robocup.org
Regras gerais: https://junior.robocup.org/robocupjunior-general-rules/

There are two kinds of wall tokens - letter victims and hazmat signs. Terms 'WT' or
'wall tokens' will be used throughout this document to refer to both hazmat signs
and letter victims > vc precisa identificar ambos enquanto navega pelo mapa e tbm deve mapeá-lo 

Mentors nao podem ficar na area do estudante nem participar/fazer construcao do robo 

* The organizers will run the games on a server-client model and prepare one RJ-45 socket for teams to connect to the game server. Teams must prepare a computer and an ethernet cable to run the prepared programs

O robô começa de um dos tiles da área 1 mais externos
Caminhos para o robô devem ter pelo menos a largura do robô e podem abrir em bifurcações mais amplas

## Areas

Areas 1 to 3 consist of a tile-based system maze-like layout. Area 4 (optional area) is not tile-based, and teams are encouraged to explore interesting simultaneous localization and mapping algorithms.
Areas sao separadas por tile de largura fixa e cor que define. Cada area é collection of tiles with a horizontal floor, a perimeter wall, and walls within the field.
Regions where the robot cannot physically traverse (i.e., openings that are smaller than the robot width) will not contain victims and hazmat signs. Such areas must be fully viewable from the opening.
5. For area 4 the course may require diagonal movement
Tiles/ladrilhos (quadrados que formam o mapa): sao 12x12cm > nao sao fisicos, eh o conceito pra criacao do mapa

Pra area 2 e 3 tem quarter-tiles que sao divisoes do tile em 4 partes iguais de 6×6cm

* Area 1: Walls are placed on the edges of each tile.
* Area 2: Walls can be placed on the edges of each quarter tile.
* Area 3: Walls can be placed on the edges of each quarter tile. Organizers can round a 90-degree corner into
* quarter circle.
* Area 4: This area’s layout is not based on a tile system, meaning walls and obstacles are not placed according to a grid system (i.e., arbitrarily).

### Passages

Passagem entre as áreas (1, 2, 3 e 4) são coloridos de forma distinta e são 1 tile de tamanho padrão e 2 paredes para não ser ambíguo a entrada/saída. São as cores das passagens:

* Entre 1 e 2: Blue
* Entre 2 e 3: Purple
* Entre 1 e 3: Yellow
* Entre 3 e 4: Red
* Entre 4 e 1: Green
* Entre 1 e 4: Orange

### Classificação dos tiles

* linear tiles: dá pra alcançar o tile inicial consistentemente seguindo a parede + à esquerda ou + à direita
        logo, estão em contato com as paredes mais externas
* floating tiles: não dá pra alcançar
    OBS: black holes mudam essa classificação, pois são considerados virtual walls (em volta são linear tiles)

## Checkpoints

* São os tiles silver/cinzas, espelhados aleatoriamente
* Area 4 vai ter um imediatamente após a passage para ela
