#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <unistd.h>

/* compile with:

	g++ test.cpp -std=c++11 -lpthread

*/

using namespace std;

pthread_mutex_t mutex_shared_number = PTHREAD_MUTEX_INITIALIZER;
int shared_number = 0;

void add_six() {
	pthread_mutex_lock( &mutex_shared_number );
	shared_number += 6;
	cout << shared_number << endl;
	pthread_mutex_unlock( &mutex_shared_number );
}

void sub_one() {
	pthread_mutex_lock( &mutex_shared_number );
	shared_number--;
	cout << shared_number << endl;
	pthread_mutex_unlock( &mutex_shared_number );
}

void* loop (void* arg) {
	for (int i = 0; i < 10; i++) {
		add_six();
		sleep(1);
	} 
}


int main() {

	pthread_t my_thread;
	
	
	pthread_mutex_init(&mutex_shared_number, NULL);
	
	// arguments:
	//	p_thread
	//	void* ?
	//	void* ?	= a function to be called
	//	void* 	= the argument of the function that is called
	int temp = pthread_create(&my_thread, NULL, loop, NULL);
	
	for (int i = 1; i < 10; i++) {
		sub_one();
		sleep(2);
	}
	
	pthread_join(my_thread, NULL);

		
	return 0;

}