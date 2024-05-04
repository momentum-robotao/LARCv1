# Ambiente Erebus > simulador

## Sobre a competição

Regras resumidas e 2023: https://erebus.rcj.cloud/docs/rules/
Regras atualizadas e 2024: https://junior.robocup.org/wp-content/uploads/2023/10/RCJRescueSimulation2024RulesDraft.pdf

## Instalação

Instale o WeBots e Erebus, para linux: https://erebus.rcj.cloud/docs/installation/linux/

## Executando o WeBots com extensão Erebus

Run the world1.wbt file from terminal. If the file located in ```/home/USER_NAME/EREBUS_FOLDER/game/worlds/world1.wbt```, just run the following command:

```bash
webots '/home/USER_NAME/EREBUS_FOLDER/game/worlds/world1.wbt'
```

### Primeira execução

The first time you run the simulator, it will automatically install the Python libraries needed to run the simulation, which may take some time (Initializing Process).

If the time limit is not displayed, as in the following image [on the website], the initializing process is in progress. This process can take up to a minutes, depending on the performance of your computer.

### Caso não apareça controles de pausar e prosseguir a simulação

Competition Controller does not appear

- If the Scene Tree(listinha dos objetos da simulação) is not displayed, go to Tools –> Scene Tree in the top menu to display it.
- Find “DEF MAINSUPERVISOR Robot” in the Scene Tree and right-click it. Click on “Show Robot Window”. > nela, em setting marca keep controller/robots...

## Partes

- Robô: você cria a sua máquina
  - peças: você pode montá-lo com os sensores/câmeras que desejar, mas a competição tem um limite de valor a ser usado sendo que cada peça vale uma quantidade determinada, então dentro disso você coloca os que quiser na posição que quiser e julgar mais interessantes
  - código: a lógica que fará ele pontuar e passar pelos desafios/objetivos da competição
- Mapa
  - Há níveis de dificuldades na competição, mas você pode criar e exportar o seu para testar o seu robô em diversas situação, aí na competição em si é gerado um aleatório condizente com o nível
  
## Criar mapa

Criador: https://osaka.rcj.cloud/service/editor/simulation/2024

## Criar um robô

[Sobre a estrutura do robô e possíveis componentes](https://erebus.rcj.cloud/docs/tutorials/the-robot/)

### Construí-lo

Site pra montar: https://v24.robot.erebus.rcj.cloud/

### Codá-lo

Informações das bibliotecas em docs.md

### Usá-lo na arena

Eles são denominados controllers, cria um arquivo e na Robot window tem a opção de load na parte controllers que você consegue criar o seu > um código em python.
Para configurar qual python ele irá usar para interpretar seu código: tools > preferences > python command (pode ser tipo ```python``` ou tipo ```/home/[user]/.../venv/bin/python3.8```).

## TODO

[] - Aprender básico do Erebus
[] - Ler as regras da competição e dos mapas

[] - Aprender encoder e giroscópio
[] - fazer robo girar 90 graus usando giroscopio, depois usando encoder
[] - fazer um seguidor de parede e entender o seguidor do codigo
