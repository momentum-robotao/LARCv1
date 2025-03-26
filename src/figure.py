import matplotlib
matplotlib.use('Agg')  # Usar backend sem interface gráfica
import matplotlib.pyplot as plt
import seaborn as sns
import numpy as np
import requests

contador = True

with open("./ngrok.txt", "r") as file:
    NGROK_URL = file.readlines()[0]
    print(f"Url do ngrok recuperada: {NGROK_URL}")
# URL do endpoint de upload no servidor Flask
url = NGROK_URL + "/arquivos"
file_path = '/tmp/grafico.png'



class Figure:
    def __init__(self):
        try:
            self.figure = plt.figure(figsize=(10,10))   
            #print("Cheguei Aqui 1")
        except Exception as e:
            print(f"Erro durante a inicialização: {e}")

    def plott(self, listX: list, listY: list, label: str, color: str) -> None:
        listX = np.array(listX)  # Converte lista_x para um array de floats
        listY = np.array(listY)
            
        sns.set_theme(style="white")
            
        # Plota o gráfico com seaborn
        plt.scatter(x=listX, y=listY, label=label, color=color)

        # Configurações adicionais
        plt.legend()
        plt.xlabel('X coordinates')
        plt.ylabel('Y coordinates')

        try:
            # Salva o gráfico em um arquivo PNG
            plt.savefig(file_path)

            # Abrindo o arquivo em modo binário
            with open(file_path, 'rb') as file:
                # Cria um dicionário com o arquivo
                files = {'file': file}
                # Envia a requisição POST com o arquivo
                response = requests.post(url, files=files)

            # Exibe a resposta do servidor
            #print(response.json())

            #print("Gráfico salvo como 'grafico.png'")
            plt.clf()
        except Exception as e:
            print(f"Erro durante a exibição do gráfico: {e}")
 
    def multi_plott(self, listX1: list, listY1: list, label1: str, color1: str, listX2: list, listY2: list, label2: str, color2: str, robot_position : list, label3 : str, color3: str) -> None:
        listX1 = np.array(listX1)  # Converte lista_x para um array de floats
        listY1 = np.array(listY1)
        listX2 = np.array(listX2)
        listY2 = np.array(listY2)
        robot_position_X = []
        robot_position_Y = []
        robot_position_X.append(np.array(robot_position)[0])
        robot_position_Y.append(np.array(robot_position)[1])

            
        sns.set_theme(style="white")
            
        # Plota o gráfico com seaborn
        plt.scatter(x=listX1, y=listY1, label=label1, color=color1)
        plt.scatter(x=listX2, y=listY2, label=label2, color=color2)
        plt.scatter(x=robot_position_X, y=robot_position[1], label=label3, color=color3)
        

        # Configurações adicionais
        plt.legend()
        plt.xlabel('X coordinates')
        plt.ylabel('Y coordinates')
        

        try:
            # Salva o gráfico em um arquivo PNG
            plt.savefig(file_path)

            # Abrindo o arquivo em modo binário
            with open(file_path, 'rb') as file:
                # Cria um dicionário com o arquivo
                files = {'file': file}
                # Envia a requisição POST com o arquivo
                response = requests.post(url, files=files)

            # Exibe a resposta do servidor
            #print(response.json())

            #print("Gráfico salvo como 'grafico.png'")
            plt.clf()
        except Exception as e:
            print(f"Erro durante a exibição do gráfico: {e}")

# Exemplo de uso
fig = Figure()
