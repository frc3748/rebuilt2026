FROM eclipse-temurin:17-jdk

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
    git curl unzip zip ca-certificates \
    libatomic1 \
    libx11-6 libxext6 libxrender1 libxtst6 libxi6 libxrandr2 libxinerama1 libxkbcommon0 libgl1 libgtk-3-0 libasound2t64 \
    xvfb x11vnc fluxbox novnc websockify \
    && rm -rf /var/lib/apt/lists/*

RUN id -u gradle >/dev/null 2>&1 || useradd -m gradle

ENV GRADLE_USER_HOME=/home/gradle/.gradle
WORKDIR /workspace
USER gradle

CMD ["bash"]
