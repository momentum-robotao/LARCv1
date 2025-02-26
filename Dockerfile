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

RUN apt-get update -y
RUN apt-get install -y libx11-dev
RUN apt-get install -y python3-tk

RUN apt-get update && apt-get install -y curl

# Create the matplotlib configuration directory
RUN mkdir -p ~/.configure/matplotlib

# Download the matplotlibrc sample configuration
RUN curl -L https://raw.githubusercontent.com/matplotlib/matplotlib/main/matplotlibrc.sample -o ~/.configure/matplotlib/matplotlibrc

# Set the backend to tkagg in the matplotlibrc file
RUN sed -i 's/#backend : Agg/backend : tkagg/' ~/.configure/matplotlib/matplotlibrc

# Set the MATPLOTLIBRC environment variable
ENV MATPLOTLIBRC=~/.configure/matplotlib

# TODO_COMPET! Set these if it is a competition or you want to debug
COPY ./ngrok.txt ./ngrok.txt
ENV ENTRIES_BETWEEN_SENDS=100
ENV LOG_PATH=/usr/local/controller/.logs/robo.log
ENV DEBUG=True

# Run controller (using extern controller helper)
CMD webots-controller --protocol=tcp --ip-address=$EREBUS_SERVER --port=1234 --robot-name=Erebus_Bot --stdout-redirect --stderr-redirect main.py
