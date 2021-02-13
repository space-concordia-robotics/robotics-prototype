//
// Created by cedric on 2020-10-10.
//

#ifndef ROVER_STOPPABLE_H
#define ROVER_STOPPABLE_H

class Stoppable{
    public:
        virtual void eStop() = 0;
        virtual ~Stoppable() {};
};
#endif //ROVER_STOPPABLE_H
