//
// Created by cedric on 2020-10-14.
//

#ifndef ROVER_UPDATABLE_H
#define ROVER_UPDATABLE_H

class Updatable{
public:
    virtual ~Updatable();
    virtual void update(unsigned long deltaMicroSeconds) = 0;
};

#endif //ROVER_UPDATABLE_H
