import OpenGL.GL as gl
import glfw
import imgui
import sys
from imgui.integrations.glfw import GlfwRenderer
from array import array 


def impl_glfw_init():
    # Define o tamanho da janela
    width, height = 1280, 720
    window_name = "Exemplo de gráfico X e Y"  # Nome da janela

    # Inicializa o GLFW (biblioteca para criar janelas OpenGL)
    if not glfw.init():
        print("Falha ao inicializar o GLFW")
        sys.exit(1)  # Se o GLFW não conseguir inicializar, termina o programa

    # Configura as versões do OpenGL que serão usadas pela janela
    glfw.window_hint(glfw.CONTEXT_VERSION_MAJOR, 3)  # Define a versão principal do OpenGL como 3
    glfw.window_hint(glfw.CONTEXT_VERSION_MINOR, 3)  # Define a versão secundária do OpenGL como 3
    glfw.window_hint(glfw.OPENGL_PROFILE, glfw.OPENGL_CORE_PROFILE)  # Usar o perfil "Core" do OpenGL
    glfw.window_hint(glfw.OPENGL_FORWARD_COMPAT, gl.GL_TRUE)  # Habilita a compatibilidade futura com OpenGL

    # Cria a janela GLFW com as configurações especificadas
    window = glfw.create_window(width, height, window_name, None, None)
    glfw.make_context_current(window)  # Define a janela criada como a janela ativa

    # Verifica se a janela foi criada corretamente
    if not window:
        glfw.terminate()  # Se não foi possível criar a janela, termina o GLFW
        print("Falha ao criar a janela")
        sys.exit(1)  # Finaliza o programa

    return window  # Retorna a janela criada

class GUI:
    def __init__(self):
        try:
            self.window = impl_glfw_init()  # Tenta inicializar a janela GLFW
            imgui.create_context()  # Cria o contexto do ImGui
            self.impl = GlfwRenderer(self.window)  # Inicializa o renderizador do ImGui
        except Exception as e:
            print(f"Erro durante a inicialização: {e}")
            sys.exit(1)  # Finaliza o programa em caso de erro

    
    def plot(self, listX : list, listY : list):
        listX = array('f', listX)  # Converte lista_x para um array de floats
        listY = array('f', listY)


        # Processa eventos da janela (como pressionamento de teclas, cliques, etc.)
        glfw.poll_events()
        
        # Processa a entrada de teclado e mouse para o ImGui
        self.impl.process_inputs()

        # Inicia um novo quadro para o ImGui (necessário para desenhar a interface)
        imgui.new_frame()

        # Cria a interface do ImGui com um gráfico
        imgui.begin("Gráfico de X e Y")  # Inicia a janela de gráfico com o título "Gráfico de X e Y"
        
        # Plota o gráfico de linha, com os valores de Y como os dados do gráfico
        imgui.plot_lines(
            "Y = f(X)",         # Título do gráfico
            listY,            # Valores no eixo Y
            overlay_text="Y = f(X)",  # Texto sobreposto no gráfico, mostrando a função
            values_offset=0,    # Não há deslocamento de dados (começa do início)
            graph_size=(0, 200)  # Tamanho do gráfico (largura auto e altura 200px)
        )
        
        imgui.end()  # Finaliza a criação da janela gráfica

        # Limpa a tela com a cor branca
        gl.glClearColor(1.0, 1.0, 1.0, 1)  # Define a cor de fundo como branca (RGB = 1.0, 1.0, 1.0)
        gl.glClear(gl.GL_COLOR_BUFFER_BIT)  # Limpa a tela, removendo qualquer conteúdo anterior

        # Renderiza o quadro do ImGui e envia para a janela
        imgui.render()
        self.impl.render(imgui.get_draw_data())  # Renderiza os dados gerados pelo ImGui na janela

        # Troca os buffers da janela, atualizando a tela com o conteúdo renderizado
        glfw.swap_buffers(self.window)
    
    def __del__(self):
        # Quando a janela for fechada, finaliza o ImGui e o GLFW
        self.impl.shutdown()  # Finaliza o renderizador do ImGui
        glfw.terminate()  # Termina a execução do GLFW (libera recursos)
    

gui = GUI()