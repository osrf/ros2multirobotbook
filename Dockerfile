FROM rust:bullseye
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    libfontconfig1-dev \
    libgraphite2-dev \
    libharfbuzz-dev \
    libicu-dev \
    libssl-dev \
    zlib1g-dev && \
    rm -rf /var/lib/apt/lists/*

RUN cargo install mdbook mdbook-open-on-gh

WORKDIR /ros2multirobotbook
COPY . .
CMD ["bash"]

