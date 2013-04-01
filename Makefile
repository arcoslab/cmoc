NAME=arcospyu
PREFIX ?= ${HOME}/local/DIR/${NAME}
DEB_TARGET=python-arcospyu_1.0-1_all.deb

install:
	python setup.py install --prefix=${PREFIX}

xstow_install: install
	cd ${PREFIX}/../ && xstow ${NAME}

xstow_uninstall:
	cd ${PREFIX}/../ && xstow -D ${NAME} && rm -rf ${NAME}

%.deb:
	python setup.py --command-packages=stdeb.command bdist_deb

deb: deb_dist/${DEB_TARGET}

deb_install: deb_dist/${DEB_TARGET}
	cd deb_dist && sudo dpkg -i *.deb

clean:
	python setup.py clean
	rm -rf build/ deb_dist/
