# Robô - momentum

## TODO

[] - Fazer girar exatamente (é fundamental pra usar qualquer algorítmos na minha opinião)
    [] - Fazer função que vai rápido no início e só ao final vai mais devagar apenas para aumentar a precisão
    [] - Versão inicial com inertialUnit
[] - Andar 1 tile exato
[] - Juntar o código
[] - Refatorar código
    [] - Padronizar snake_case
    [] - Não usar variável global
    [] - Tirar print's (usa variável de ambiente de DEBUG)
    [] - Constante em UPPER_CASE
[] - Fazer requirements.txt
[] - No caso de ter DFS e em uma direção ele marca posição como sem parede, mas na verdade depois tenta acessá-la por lugar que tem parede, dá problema

## Resolver algum dia

Flood fill faster: BFS, DFS, flood fill, DSU
Mapear todas as distências pra descobrir formato da sala em O(1) não saberia mais rápido o caminho
Pior caso, em matrix m \* n, só BFS/DFS ficaria: O(2 \* m \* n) tiles percorridos
Correção de erros: https://www.youtube.com/watch?v=-XU54IsG8Vo

BFS pra encurtar caminho pra voltar da DFS

Reconhecer com IA/TensorFlow treinado as vítimas
