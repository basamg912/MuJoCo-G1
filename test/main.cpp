#include <iostream>
#include <string>
#include <mujoco/mujoco.h>

using namespace std;
int main(){
    int version = mj_version();
    const char* G1_XML_PATH{"/Users/basamg/work/Mujoco_Humanoid/model/g1/g1_23dof.xml"};
    char* err;
    int error_size = sizeof(err);
    mjModel* m = mj_loadXML(G1_XML_PATH, nullptr, err, error_size);
    mjData* d = mj_makeData(m);
    cout << d->time << endl;
    while(d->time <= 1){
        mj_step(m,d);
        cout << d->time << endl;
    }
    return 0;
}
