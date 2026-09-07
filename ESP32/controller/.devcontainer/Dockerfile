# Pin to the same ESP-IDF version as .github/workflows/esp32-build.yml and
# the README ("ESP-IDF v5.5.4"). Bluepad32 does not yet document/guarantee
# support for ESP-IDF v6+, so do not default this to "latest".
ARG DOCKER_TAG=v5.5.4
FROM espressif/idf:${DOCKER_TAG}

ENV LC_ALL=C.UTF-8
ENV LANG=C.UTF-8

RUN apt-get update -y && apt-get install udev -y

RUN echo "source /opt/esp/idf/export.sh > /dev/null 2>&1" >> ~/.bashrc

ENTRYPOINT [ "/opt/esp/entrypoint.sh" ]

CMD ["/bin/bash", "-c"]