FROM python:3.11-slim AS builder

ENV PIP_DISABLE_PIP_VERSION_CHECK=1 PIP_NO_CACHE_DIR=1
WORKDIR /srv/wt-em
RUN apt-get update && apt-get install -y --no-install-recommends build-essential cargo rustc \
    && rm -rf /var/lib/apt/lists/*
COPY requirements.txt requirements-plotter.txt ./
RUN python -m pip install -r requirements-plotter.txt
COPY scripts ./scripts
COPY native ./native
COPY app ./app
ARG SOURCE_COMMIT=development
RUN python -c "import json; from pathlib import Path; Path('build-info.json').write_text(json.dumps({'commit': '$SOURCE_COMMIT'}))"
RUN python scripts/bootstrap_runtime.py
RUN python scripts/build_em_backend.py
RUN python scripts/build_missile_backend.py
RUN python scripts/build_rust_backend.py

FROM python:3.11-slim

ENV PYTHONUNBUFFERED=1 \
    OPENBLAS_NUM_THREADS=1 \
    OMP_NUM_THREADS=1 \
    MKL_NUM_THREADS=1 \
    WT_EM_HOST=0.0.0.0 \
    PORT=8080 \
    WT_EM_OUTPUT_DIR=/data/em \
    MPLCONFIGDIR=/tmp/matplotlib
WORKDIR /srv/wt-em
RUN useradd --create-home --uid 10001 emserver && mkdir -p /data/em /tmp/matplotlib \
    && chown -R emserver:emserver /data /tmp/matplotlib
COPY --from=builder /usr/local /usr/local
COPY --from=builder /srv/wt-em /srv/wt-em
USER emserver
EXPOSE 8080
CMD ["python", "scripts/em_server.py"]
