# Robô - momentum

## Guia

### Funcionamentos

#### LIDAR

Medições das distâncias são uniformemente espaçadas ao longo do campo de visão dele. O começo do ângulo (ângulo 0) varia em relação ao robô de acordo com o posicionamento deste, então cresce o ângulo positivamente no sentido horário e em relação enquanto você percorre a lista das distâncias retornadas pelo LIDAR.
i-ésimo ponto: `angle = min_fov + i * (fov / horizontal_resolution)`

### Para rodar

Setando variáveis de ambiente: no docker elas ficam setadas no `DockerFile`, já ao rodar em ambiente local dê export NOME_VARIAVEL_AMBIENTE=valor_dela

#### Em ambiente de teste

#### Em ambiente de competição

!!! IMPORTANTE
Não setar a variável de ambiente `DEBUG`, pois poderá levar a comportamentos indesejados que facilitariam o reconhecimento de erro, mas podem por exemplo parar o robô

## Resolver algum dia

Pior caso, em matrix m \* n, só BFS/DFS ficaria: O(2 \* m \* n) tiles percorridos
Correção de erros: https://www.youtube.com/watch?v=-XU54IsG8Vo

BFS pra encurtar caminho pra voltar da DFS

se necessário para SLAM, há pointCloud do lidar aqui: https://erebus.rcj.cloud/docs/tutorials/sensors/lidar/, apenas o processamento que é levemente custoso para esse método

Talvez usar multi-threading ou multi-processing, pode ter main Thread que movimenta o robô (e lê os sensores, se alguns sensores ficarem lentos, algo que podemos tentar, mas não é o ideal é colocá-los em um processo/thread separada também) e a que faz reconhecimento de vítima
- https://github.com/cyberbotics/webots/issues/3028
- https://www.youtube.com/watch?v=AZnGRKFUU0c
- código do André se precisar
- https://www.youtube.com/watch?v=GT10PnUFLlE
