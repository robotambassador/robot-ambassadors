/**
 * @file /eros_boost/src/shared_pointer.cpp
 *
 * @brief Test the boost compilation/install.
 *
 * @date Nov 17, 2010
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <iostream>
#include <boost/shared_ptr.hpp>
#include <boost/thread/thread.hpp>

/*****************************************************************************
 * Class
 ****************************************************************************/

struct Worker {

    Worker() : count(0) {}
    void operator()() {
        while ( count < 10 ) {
            ++count;
            std::cout << "Count: " << count << std::endl;
        }
    }
    int count;
};

/*****************************************************************************
 * ** Main
 * *****************************************************************************/

int main(int argc, char **argv) {
    boost::shared_ptr<int> ptr(new int(3));
    std::cout << "Int: " << *ptr << std::endl;
    Worker worker;
    boost::thread thrd(worker);
    thrd.join();
    std::cout << "Finishing" << std::endl;
    return 0;
}
