FROM alfredroberts/erebus

# Copy needed files to container
RUN mkdir -p /usr/local/controller
WORKDIR /usr/local/controller

COPY ./src/ .
# COPY ./requirements.txt ./requirements.txt

# Install dependencies
# RUN pip3 install -r requirements.txt
COPY ./venv/lib64/python3.12/site-packages/ .

# Set env vars
ENV DEBUG=True
ENV LOG_PATH=/usr/local/controller/.logs/robo.log
ENV TIME_STEP=32

# Run controller (using extern controller helper)
CMD webots-controller --protocol=tcp --ip-address=$EREBUS_SERVER --port=1234 --robot-name=Erebus_Bot --stdout-redirect --stderr-redirect main.py
