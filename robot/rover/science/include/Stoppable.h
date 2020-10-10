//
// Created by cedric on 2020-10-10.
//

#ifndef ROVER_STOPPABLE_H
#define ROVER_STOPPABLE_H

class Stoppable{
    public:
        virtual ~Stoppable();
        virtual void eStop() = 0;
};
#endif //ROVER_STOPPABLE_H
