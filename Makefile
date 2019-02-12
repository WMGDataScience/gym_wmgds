.PHONY: install test

install:
	pip install -r requirements.txt

base:
	docker pull ubuntu:14.04
	docker tag ubuntu:14.04 quay.io/openai/gym_wmgds:base
	docker push quay.io/openai/gym_wmgds:base

test:
	docker build -f test.dockerfile -t quay.io/openai/gym_wmgds:test .
	docker push quay.io/openai/gym_wmgds:test

upload:
	rm -rf dist
	python setup.py sdist
	twine upload dist/*

docker-build:
	docker build -t quay.io/openai/gym_wmgds .

docker-run:
	docker run -ti quay.io/openai/gym_wmgds bash
