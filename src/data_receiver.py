import ngrok
from flask import Flask, request

app = Flask(__name__)

PORT = 5555


@app.route("/start_simulation", methods=["POST"])
def start_simulation():
    print("Starting simulation")

    return "ok"


@app.route("/send", methods=["POST"])
def send_data():
    print("Received", request.json)
    return "ok"


if __name__ == "__main__":
    listener = ngrok.forward(PORT, authtoken_from_env=True)
    print(f"Listening at {listener.url()}")

    with open("./ngrok.txt", "w+") as file:
        file.write(listener.url())

    app.run("127.0.0.1", port=PORT)
