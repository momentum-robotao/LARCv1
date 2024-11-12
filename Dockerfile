FROM alfredroberts/erebus

# Copy needed files to container
RUN mkdir -p /usr/local/controller
WORKDIR /usr/local/controller

COPY ./src/ .
COPY ./requirements.txt ./requirements.txt

# Install dependencies
RUN pip3 install -r requirements.txt

# Set env vars
ENV LOG_PATH=/usr/local/controller/.logs/robo.log
ENV TIME_STEP=32
# Configure
# TIME_TOLERANCE: se tempo restante (real ou simulado) for inferior a isso, o mapa é enviado 
ENV TIME_TOLERANCE=3
# IMPORTANT! Remove these DEBUG env vars during competition
COPY ./ngrok.txt ./ngrok.txt

ENV DEBUG=True
ENV ON_DOCKER=True
ENV ENTRIES_BETWEEN_SENDS=100
ENV ANGLE_MAX_DIFFERENCE=0.3


# Run controller (using extern controller helper)
CMD webots-controller --protocol=tcp --ip-address=$EREBUS_SERVER --port=1234 --robot-name=Erebus_Bot --stdout-redirect --stderr-redirect main.py
