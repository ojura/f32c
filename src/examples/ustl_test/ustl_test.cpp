#include <ualgo.h>
#include <ustring.h>
#include <uvector.h>
#include <umap.h>
#include <stdio.h>
#include <stdlib.h>

void printVector_int(ustl::vector<int> &v) {
    printf("[ ");
    for(int i : v) {
        printf("%d ", i);
    }
    printf("]\n");
}

int main() {
    
    ustl::string test("Hello everyone!");
    
    ustl::vector<int> matr(4,0);
    
    //ustl::map<ustl::string, ustl::string> uloga;
    //uloga["marko"] = "tata";
    //uloga["ivan"] = "brat";
    //uloga["vinko"] = "ujak";
    
    
    matr[0] = 3; matr[1] = 4; matr[2] = 1; matr[3] = 2;
    printf("%s Size: %d Contents: ", test.c_str(), matr.size()); printVector_int(matr);
    
    struct cmp {int operator()(int a, int b) { return a < b; } };
    ustl::sort(matr.begin(), matr.end(), cmp());
    
    printf("%s Sorted: Size: %d Contents: ", test.c_str(), matr.size()); printVector_int(matr);
    
    matr.push_back(157);
    printf("%s push_back(157): Size: %d Contents: ", test.c_str(), matr.size());  printVector_int(matr);
    
    while(1) { };

}
