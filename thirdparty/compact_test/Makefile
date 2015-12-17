wgs84_to_ltp: wgs84_to_ltp.cpp
	g++ -o wgs84_to_ltp  wgs84_to_ltp.cpp -L/usr/local/lib -lGeographic

controller: controller.cpp
	g++ -o controller  controller.cpp -lm

test_frametransform: test_frametransform.cpp
	g++ -o test_frametransform  test_frametransform.cpp -lm

all: wgs84_to_ltp controller test_frametransform

clean:
	rm -rf *.o wgs84_to_ltp controller test_frametransform
