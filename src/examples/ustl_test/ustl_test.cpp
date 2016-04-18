#include <ualgo.h>
#include <ustring.h>
#include <uvector.h>
#include <umap.h>
#include <stdio.h>
#include <stdlib.h>



int main() {
	
	printf("Hello!\n");
	ustl::string test("Bok svima!");

	ustl::vector<int> matr(4,0);

	ustl::map<ustl::string, ustl::string> uloga;

	uloga["marko"] = "tata";
	uloga["ivan"] = "brat";
	uloga["vinko"] = "ujak";

	matr[0] = 4; matr[1] = 3; matr[2] = 2; matr[3] = 1;
	matr.push_back(74);
		
	struct cmp {int operator()(int a, int b) { return a < b; } };
	//ustl::sort(matr.begin(), matr.end(), cmp());

	printf("Hi! %s Size:  %d, %d %d %d %d %d\n", test.c_str(), matr.size(), matr[0], matr[1], matr[2],matr[3],matr[4]);
	
	while(1) { };
}
