// Exercise the production target lookup and its actual typed::Bail contract.
#include "controls.h"
#include "typed/typed_flight.h"
#include <cassert>
#include <iostream>
int main() {
    typed::SteeredPath path;
    path.position={0,5000,0};path.velocity={250,0,0};path.quaternion={0,0,0,1};
    typed::TargetMotion motion{};motion.steered=&path;
    assert(typed::target_at(motion,0).position[1]==5000);
    int rejected=0;
    for (double time:{typed::STEP,1.,300.}) {
        try {typed::target_at(motion,time);}
        catch(const typed::Bail& error) {assert(std::strstr(error.reason,"ahead"));++rejected;}
    }
    assert(rejected==3);
    path.advance(0,typed::STEP,0,0,0);
    assert(typed::target_at(motion,typed::STEP).position[0]>0);
    try {typed::target_at(motion,typed::add(typed::STEP,typed::STEP));}
    catch(const typed::Bail&) {++rejected;}
    assert(rejected==4);
    std::cout << "Typed look-ahead: 4 rejected; 2 legal lookups passed\n";
}
