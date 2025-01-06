import os
from datetime import datetime

import ngrok
from flask import Flask, request

app = Flask(__name__)

PORT = 5555
LOG_PATH = os.getenv("LOG_PATH", "./.logs/log.txt")


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


if __name__ == "__main__":
    listener = ngrok.forward(PORT, authtoken="2ocKuROdaZN441lY71sJhk1BKD6_7w3aicbKWTkzwF9ev3Cbj")
    print(f"Listening at {listener.url()}")

    with open("./ngrok.txt", "w+") as file:
        file.write(listener.url())

    app.run("127.0.0.1", port=PORT)
