import os
from datetime import datetime

import ngrok
from flask import Flask, request, jsonify, send_from_directory 

app = Flask(__name__)

PORT = 5555
LOG_PATH = os.getenv("LOG_PATH", "./.logs/log.txt")

# Defina o diretório de upload
UPLOAD_FOLDER = '/home/alunoc/git_projects/LARCv1/SLAM_IMAGES' # PC DO ETAPA
#UPLOAD_FOLDER = r"C:\Users\Pichau\Des  ktop\LARCV1\SLAM_IMAGES"  #Meu PC
app.config['UPLOAD_FOLDER'] = UPLOAD_FOLDER

# Certifique-se de que o diretório de upload existe
os.makedirs(app.config['UPLOAD_FOLDER'], exist_ok=True)

@app.route("/start_simulation", methods=["POST"])
def start_simulation():
    print("Starting new simulation")
    with open(LOG_PATH, "w+") as file:
        file.write(f"Nova execução, apagando dados... {datetime.now()}\n")
    return "ok"


@app.route("/send", methods=["POST"])
def send_data():
    with open(LOG_PATH, "a+") as file:
        file.write(request.json["new_entries"])
    return "ok"

@app.route("/arquivos", methods=["POST"])
def post_arquivo():
    if 'file' not in request.files:
        return jsonify({'error': 'Nenhum arquivo enviado'}), 400

    file = request.files['file']
    if file.filename == '':
        return jsonify({'error': 'Nome de arquivo inválido'}), 400

    filename = file.filename
    file_path = os.path.join(app.config['UPLOAD_FOLDER'], filename)
    file.save(file_path)
    
    
    return jsonify({'message': 'Arquivo recebido com sucesso', 'filename': filename}), 200


if __name__ == "__main__":
    listener = ngrok.forward(PORT, authtoken="2nRY8dp1IQp7JqwtZJuvbVcW90K_3P1eoms4QKsX7K4wCtajm")
    print(f"Listening at {listener.url()}")

    with open("./ngrok.txt", "w+") as file:
        file.write(listener.url())

    app.run("127.0.0.1", port=PORT)