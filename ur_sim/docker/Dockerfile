FROM universalrobots/ursim_e-series

ENV ROBOT_MODEL=UR5 

# VNC port: 5900
EXPOSE 5900
# Web browser VNC port: 6080
EXPOSE 6080
# Interface Ports
EXPOSE 29999
EXPOSE 30001-30004

COPY externalcontrol-1.0.5.urcap /urcaps/externalcontrol-1.0.5.jar
