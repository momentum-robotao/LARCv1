import matplotlib
matplotlib.use('Agg')  # Usar backend sem interface gráfica
import matplotlib.pyplot as plt
import seaborn as sns
import numpy as np

contador = True

class Figure:
    def __init__(self):
        try:
            self.figure = plt.figure(figsize=(10,10))   
            print("Cheguei Aqui 1")
        except Exception as e:
            print(f"Erro durante a inicialização: {e}")

    def plott(self, listX: list, listY: list, label: str, color: str = 'blue'):
        global contador
        if contador : 
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
                plt.savefig('/tmp/grafico.png')
                print("Gráfico salvo como 'grafico.png'")
                plt.clf()
            except Exception as e:
                print(f"Erro durante a exibição do gráfico: {e}")
            
            contador = False
        else : 
            print("Hello WOrld")

# Exemplo de uso
fig = Figure()
