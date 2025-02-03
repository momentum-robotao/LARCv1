FROM alfredroberts/erebus

# Copy needed files to container
RUN mkdir -p /usr/local/controller
WORKDIR /usr/local/controller

# Copy code
COPY ./src/ .

# Install dependencies
COPY ./requirements.txt ./requirements.txt
RUN pip3 install -r requirements.txt

# Set settings env vars
# ? se tempo restante (real ou simulado) for inferior a isso, o mapa é enviado
ENV TIME_TOLERANCE=3
ENV ANGLE_MAX_DIFFERENCE=0.3
ENV TIME_STEP=32

COPY ./ngrok.txt ./ngrok.txt
ENV ENTRIES_BETWEEN_SENDS=1000
ENV LOG_PATH=/usr/local/controller/.logs/robo.log
ENV DEBUG=False

# Run controller (using extern controller helper)
CMD webots-controller --protocol=tcp --ip-address=$EREBUS_SERVER --port=1234 --robot-name=Erebus_Bot --stdout-redirect --stderr-redirect main.py
