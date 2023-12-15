FROM python:3.9-slim-buster

RUN mkdir /app
WORKDIR /app

COPY requirements.txt .
RUN pip install -i https://pypi.tuna.tsinghua.edu.cn/simple -r requirements.txt

COPY . .
EXPOSE 8000
CMD \
    python manage.py migrate && \
    python manage.py runserver --noreload 0.0.0.0:8000


