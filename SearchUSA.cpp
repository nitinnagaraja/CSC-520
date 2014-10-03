#include <iostream>
#include <cstring>
#include <cmath>
#include <algorithm>
#include <deque>

using namespace std;

#define PI 3.141593
#define NUMCITIES 150
#define NUMROADS 300

struct city
{
  string name;
  double latitude, longitude;
};

struct road
{
  city *start, *end;
  double distance;
};

struct path
{
  path *prev;
  city *end;
  double cost;
};

struct path_name {
  path* item;
  path_name(path* p) { item = p; }
  bool operator()(path* p) {
    bool val = (p->end->name == item->end->name);
    return val;
  }
};

struct solution {
  path* p;
  deque<path*> closed;  
};

city* city_list[NUMCITIES];
int current_city=0;
city* goal_city;

road* road_list[NUMROADS];
int current_road=0;

void populate_city_list();
void populate_road_list();

void print_usage_help()
{
  cout << "Please enter arguments in the following format:\n";
  cout << "./a.out searchtype srccityname destcityname where,\n";
  cout << "\tsearchtype is one of astar, greedy or dynamic\n";
  cout << "\tsrccityname is the name of the source city\n";
  cout << "\tdestcityname is the name of the destination city\n";
  return;
}

void create_city(string name, double latitude, double longitude)
{  
  city* c = new city;
  city_list[current_city] = c;
  c->name.assign(name);
  c->latitude = latitude;
  c->longitude = longitude;
  current_city++;
}

city* find_city(string name)
{
  int i = 0;
  city *start_city = NULL;
  for(i = 0 ; i < NUMCITIES ; i++)
    if(city_list[i] != NULL && city_list[i]->name == name)
      start_city = city_list[i];
  return start_city;
}

void create_road(string name1, string name2, int distance)
{
  city *start_point = find_city(name1);
  city *end_point = find_city(name2);
  
  if(start_point == NULL || end_point == NULL) {
    cerr << "Can't create a road involving a null city " << current_road << "\n";
    return;
  }

  road *r1 = new road;
  road_list[current_road] = r1;
  r1->start = start_point;
  r1->end = end_point;
  r1->distance = distance;
  current_road++;

  road *r2 = new road;
  road_list[current_road] = r2;
  r2->start = end_point;
  r2->end = start_point;
  r2->distance = distance;
  current_road++;
}

int path_node_count(path* p) {
  int i = 0;
  while(p != NULL) {
    p = p->prev;
    i++;
  }
  return (i - 1);
}

double distance(city* a, city* b)
{
  double average_latitude, longitude_distance;
  average_latitude = (a->latitude + b->latitude) / 2;
  return (sqrt((69.5 * pow((a->latitude - b->latitude), 2)) +
          pow((69.5 * cos(average_latitude/180 * PI) * (a->longitude - b->longitude)), 2)));
}

deque<path*> get_neighbours(path* cur) {
  int i = 0;
  deque<path*> n;
  for (i = 0 ; i < NUMROADS ; i++) {
    road *r = road_list[i];
    if (r != NULL && r->start->name == cur->end->name) {
      path *p = new path;
      p->end = r->end;
      p->prev = cur;
      p->cost = cur->cost + r->distance;
      n.push_back(p);
    }
  }
  return n;
}

void print_queue(deque<path*> list) {
  deque<path *>::iterator i;
  for(i = list.begin() ; i != list.end() ; i++)
    cout << (*i)->end->name << ", ";
  cout << "\n";
  cout.flush();
}

void greedy_add_paths(deque<path*> &open, deque<path*> &closed, deque<path*> paths) {
  deque<path *>::iterator i, j;
  
  for(i = paths.begin() ; i != paths.end() ; i++) {
    j = find_if(closed.begin(), closed.end(), path_name(*i));
    if (j != closed.end())
      continue;
    j = find_if(open.begin(), open.end(), path_name(*i));
    if (j == open.end()) {
      open.push_back(*i);
    }
  }
}

void add_paths(deque<path*> &open, deque<path*> &closed, deque<path*> paths) {
  deque<path*>::iterator i,j;
  
  for(i = paths.begin(); i != paths.end() ; i++) {
    j = find_if(closed.begin(), closed.end(), path_name(*i));
    if (j != closed.end())
      continue;
    j = find_if(open.begin(), open.end(), path_name(*i));
    if (j != open.end()) {
      if ((*i)->cost < (*j)->cost) {
        open.erase(j);
        open.push_back(*i);
      } else
          continue;
    } else {
      open.push_back(*i);
    }
  }
}

bool astar_compare(path* p1, path* p2) {
  return ((p1->cost + distance(p1->end, goal_city)) < (p2->cost + distance(p2->end, goal_city)));
}

solution *astar(city* start, city* dest) {
  deque<path*> open,closed;
  goal_city = dest;

  path* s = new path;
  s->prev = NULL;
  s->end = start;
  s->cost = 0;
  open.push_back(s);

  while(!open.empty()) {
    path *cur = open.front();
    open.pop_front();
    closed.push_back(cur);

    if (cur->end->name == goal_city->name) {
      solution* s = new solution;
      s->p = cur;
      s->closed = closed;
      return s;
    }

    deque<path*>n = get_neighbours(cur);

    add_paths(open, closed, n);

    sort(open.begin(), open.end(), astar_compare);
  }
  cerr << "Couldn't find path\n";
  exit(1);
}

bool dynamic_compare(path* p1, path* p2) {
  return (p2->cost < p2->cost);
}

solution* dynamic(city* start, city* dest) {
  deque<path*> open,closed;
  goal_city = dest;

  path* s = new path;
  s->prev = NULL;
  s->end = start;
  s->cost = 0;
  open.push_back(s);

  while(!open.empty()) {
    path *cur = open.front();
    open.pop_front();
    closed.push_back(cur);

    if (cur->end->name == goal_city->name) {
      solution* s = new solution;
      s->p = cur;
      s->closed = closed;
      return s;
    }

    deque<path*>n = get_neighbours(cur);

    add_paths(open,closed,n);

    sort(open.begin(), open.end(), dynamic_compare);
  }
  cerr << "Couldn't find path\n";
  exit(1);
}

bool greedy_compare(path* p1, path* p2) {
  return (distance(p1->end, goal_city) < distance(p2->end, goal_city));
}

solution* greedy(city* start, city* dest) {
  deque<path*> open,closed;
  goal_city = dest;

  path* s = new path;
  s->prev = NULL;
  s->end = start;
  s->cost = 0;
  open.push_back(s);

  while(!open.empty()) {
    path *cur = open.front();
    open.pop_front();
    closed.push_back(cur);

    if (cur->end->name == goal_city->name) {
      solution* s = new solution;
      s->p = cur;
      s->closed = closed;
      return s;
    }

    deque<path*>n = get_neighbours(cur);

    greedy_add_paths(open,closed,n);

    sort(open.begin(), open.end(), greedy_compare);
  }
  cerr << "Couldn't find path\n";
  exit(1);
}

void print_path(path* p) {
  deque<string> nodes;
  while(p != NULL && p->end != NULL) {
    nodes.push_front(p->end->name);
    p = p->prev;
  }
  while(!nodes.empty()) {
    string n = nodes.front();
    nodes.pop_front();
    cout << n << ", ";
  }
}

void print_solution(solution* s) {
  cout << "\nExpanded nodes: "; print_queue(s->closed); cout <<"\n";
  cout << "Number of expanded nodes: " << s->closed.size() - 1 << "\n";
  cout << "----------------------------------\n";
  cout << "Nodes in the solution path: "; print_path(s->p); cout << "\n";
  cout << "Number of path nodes: " << path_node_count(s->p) << "\n";
  cout << "Path length: " << s->p->cost << "\n";
  cout << "----------------------------------\n";
}

int main (int argc, char* argv[])
{
  populate_city_list();
  populate_road_list();

  if (argc < 4 || argc > 4) {
    print_usage_help();
  } else {
    city* start = find_city(argv[2]);
    if (start == NULL)
    {
      cerr << "Error: start city could not be found.\n";
      return -1;
    }

    city* dest = find_city(argv[3]);
    if (dest == NULL)
    {
      cerr << "Error: destination city could not be found.\n";
      return -1;
    }
    
    if (strcmp(argv[1], "astar") == 0) {
      solution* s = astar(start, dest);
      print_solution(s);
    } else if (strcmp(argv[1], "greedy") == 0) {
      solution* s = greedy(start, dest);
      print_solution(s);
    } else if (strcmp(argv[1], "dynamic") == 0) {
      solution* s = dynamic(start, dest);
      print_solution(s);
    } else {
      print_usage_help();
    }
  }
  return 0;
}

void populate_city_list()
{
  int i = 0;
  for(i = 0 ; i < NUMCITIES ; i++)
    city_list[i] = NULL;

  create_city("albanyGA", 31.58, 84.17);
  create_city("albanyNY", 42.66, 73.78);
  create_city("albuquerque", 35.11, 106.61);
  create_city("atlanta", 33.76, 84.40);
  create_city("augusta", 33.43, 82.02);
  create_city("austin", 30.30, 97.75);
  create_city("bakersfield", 35.36, 119.03);
  create_city("baltimore", 39.31, 76.62);
  create_city("batonRouge", 30.46, 91.14);
  create_city("beaumont", 30.08, 94.13);
  create_city("boise", 43.61, 116.24);
  create_city("boston", 42.32, 71.09);
  create_city("buffalo", 42.90, 78.85);
  create_city("calgary", 51.00, 114.00);
  create_city("charlotte", 35.21, 80.83);
  create_city("chattanooga", 35.05, 85.27);
  create_city("chicago", 41.84, 87.68);
  create_city("cincinnati", 39.14, 84.50);
  create_city("cleveland", 41.48, 81.67);
  create_city("coloradoSprings", 38.86, 104.79);
  create_city("columbus", 39.99, 82.99);
  create_city("dallas", 32.80, 96.79);
  create_city("dayton", 39.76, 84.20);
  create_city("daytonaBeach", 29.21, 81.04);
  create_city("denver", 39.73, 104.97);
  create_city("desMoines", 41.59, 93.62);
  create_city("elPaso", 31.79, 106.42);
  create_city("eugene", 44.06, 123.11);
  create_city("europe", 48.87, 2.33);
  create_city("ftWorth", 32.74, 97.33);
  create_city("fresno", 36.78, 119.79);
  create_city("grandJunction", 39.08, 108.56);
  create_city("greenBay", 44.51, 88.02);
  create_city("greensboro", 36.08, 79.82);
  create_city("houston", 29.76, 95.38);
  create_city("indianapolis", 39.79, 86.15);
  create_city("jacksonville", 30.32, 81.66);
  create_city("japan", 35.68, 220.23);
  create_city("kansasCity", 39.08, 94.56);
  create_city("keyWest", 24.56, 81.78);
  create_city("lafayette", 30.21, 92.03);
  create_city("lakeCity", 30.19, 82.64);
  create_city("laredo", 27.52, 99.49);
  create_city("lasVegas", 36.19, 115.22);
  create_city("lincoln", 40.81, 96.68);
  create_city("littleRock", 34.74, 92.33);
  create_city("losAngeles", 34.03, 118.17);
  create_city("macon", 32.83, 83.65);
  create_city("medford", 42.33, 122.86);
  create_city("memphis", 35.12, 89.97);
  create_city("mexia", 31.68, 96.48);
  create_city("mexico", 19.40, 99.12);
  create_city("miami", 25.79, 80.22);
  create_city("midland", 43.62, 84.23);
  create_city("milwaukee", 43.05, 87.96);
  create_city("minneapolis", 44.96, 93.27);
  create_city("modesto", 37.66, 120.99);
  create_city("montreal", 45.50, 73.67);
  create_city("nashville", 36.15, 86.76);
  create_city("newHaven", 41.31, 72.92);
  create_city("newOrleans", 29.97, 90.06);
  create_city("newYork", 40.70, 73.92);
  create_city("norfolk", 36.89, 76.26);
  create_city("oakland", 37.80, 122.23);
  create_city("oklahomaCity", 35.48, 97.53);
  create_city("omaha", 41.26, 96.01);
  create_city("orlando", 28.53, 81.38);
  create_city("ottawa", 45.42, 75.69);
  create_city("pensacola", 30.44, 87.21);
  create_city("philadelphia", 40.72, 76.12);
  create_city("phoenix", 33.53, 112.08);
  create_city("pittsburgh", 40.40, 79.84);
  create_city("pointReyes", 38.07, 122.81);
  create_city("portland", 45.52, 122.64);
  create_city("providence", 41.80, 71.36);
  create_city("provo", 40.24, 111.66);
  create_city("raleigh", 35.82, 78.64);
  create_city("redding", 40.58, 122.37);
  create_city("reno", 39.53, 119.82);
  create_city("richmond", 37.54, 77.46);
  create_city("rochester", 43.17, 77.61);
  create_city("sacramento", 38.56, 121.47);
  create_city("salem", 44.93, 123.03);
  create_city("salinas", 36.68, 121.64);
  create_city("saltLakeCity", 40.75, 111.89);
  create_city("sanAntonio", 29.45, 98.51);
  create_city("sanDiego", 32.78, 117.15);
  create_city("sanFrancisco", 37.76, 122.44);
  create_city("sanJose", 37.30, 121.87);
  create_city("sanLuisObispo", 35.27, 120.66);
  create_city("santaFe", 35.67, 105.96);
  create_city("saultSteMarie", 46.49, 84.35);
  create_city("savannah", 32.05, 81.10);
  create_city("seattle", 47.63, 122.33);
  create_city("stLouis", 38.63, 90.24);
  create_city("stamford", 41.07, 73.54);
  create_city("stockton", 37.98, 121.30);
  create_city("tallahassee", 30.45, 84.27);
  create_city("tampa", 27.97, 82.46);
  create_city("thunderBay", 48.38, 89.25);
  create_city("toledo", 41.67, 83.58);
  create_city("toronto", 43.65, 79.38);
  create_city("tucson", 32.21, 110.92);
  create_city("tulsa", 36.13, 95.94);
  create_city("uk1", 51.30, 0.00);
  create_city("uk2", 51.30, 0.00);
  create_city("vancouver", 49.25, 123.10);
  create_city("washington", 38.91, 77.01);
  create_city("westPalmBeach", 26.43, 80.03);
  create_city("wichita", 37.69, 97.34);
  create_city("winnipeg", 49.90, 97.13);
  create_city("yuma", 32.69, 114.62);
}

void populate_road_list()
{
  int i = 0;
  for(i = 0 ; i  <NUMROADS ; i++)
    road_list[i] = NULL;

  create_road("albanyNY", "montreal", 226);
  create_road("albanyNY", "boston", 166);
  create_road("albanyNY", "rochester", 148);
  create_road("albanyGA", "tallahassee", 120);
  create_road("albanyGA", "macon", 106);
  create_road("albuquerque", "elPaso", 267);
  create_road("albuquerque", "santaFe", 61);
  create_road("atlanta", "macon", 82);
  create_road("atlanta", "chattanooga", 117);
  create_road("augusta", "charlotte", 161);
  create_road("augusta", "savannah", 131);
  create_road("austin", "houston", 186);
  create_road("austin", "sanAntonio", 79);
  create_road("bakersfield", "losAngeles", 112);
  create_road("bakersfield", "fresno", 107);
  create_road("baltimore", "philadelphia", 102);
  create_road("baltimore", "washington", 45);
  create_road("batonRouge", "lafayette", 50);
  create_road("batonRouge", "newOrleans", 80);
  create_road("beaumont", "houston", 69);
  create_road("beaumont", "lafayette", 122);
  create_road("boise", "saltLakeCity", 349);
  create_road("boise", "portland", 428);
  create_road("boston", "providence", 51);
  create_road("buffalo", "toronto", 105);
  create_road("buffalo", "rochester", 64);
  create_road("buffalo", "cleveland", 191);
  create_road("calgary", "vancouver", 605);
  create_road("calgary", "winnipeg", 829);
  create_road("charlotte", "greensboro", 91);
  create_road("chattanooga", "nashville", 129);
  create_road("chicago", "milwaukee", 90);
  create_road("chicago", "midland", 279);
  create_road("cincinnati", "indianapolis", 110);
  create_road("cincinnati", "dayton", 56);
  create_road("cleveland", "pittsburgh", 157);
  create_road("cleveland", "columbus", 142);
  create_road("coloradoSprings", "denver", 70);
  create_road("coloradoSprings", "santaFe", 316);
  create_road("columbus", "dayton", 72);
  create_road("dallas", "denver", 792);
  create_road("dallas", "mexia", 83);
  create_road("daytonaBeach", "jacksonville", 92);
  create_road("daytonaBeach", "orlando", 54);
  create_road("denver", "wichita", 523);
  create_road("denver", "grandJunction", 246);
  create_road("desMoines", "omaha", 135);
  create_road("desMoines", "minneapolis", 246);
  create_road("elPaso", "sanAntonio", 580);
  create_road("elPaso", "tucson", 320);
  create_road("eugene", "salem", 63);
  create_road("eugene", "medford", 165);
  create_road("europe", "philadelphia", 3939);
  create_road("ftWorth", "oklahomaCity", 209);
  create_road("fresno", "modesto", 109);
  create_road("grandJunction", "provo", 220);
  create_road("greenBay", "minneapolis", 304);
  create_road("greenBay", "milwaukee", 117);
  create_road("greensboro", "raleigh", 74);
  create_road("houston", "mexia", 165);
  create_road("indianapolis", "stLouis", 246);
  create_road("jacksonville", "savannah", 140);
  create_road("jacksonville", "lakeCity", 113);
  create_road("japan", "pointReyes", 5131);
  create_road("japan", "sanLuisObispo", 5451);
  create_road("kansasCity", "tulsa", 249);
  create_road("kansasCity", "stLouis", 256);
  create_road("kansasCity", "wichita", 190);
  create_road("keyWest", "tampa", 446);
  create_road("lakeCity", "tampa", 169);
  create_road("lakeCity", "tallahassee", 104);
  create_road("laredo", "sanAntonio", 154);
  create_road("laredo", "mexico", 741);
  create_road("lasVegas", "losAngeles", 275);
  create_road("lasVegas", "saltLakeCity", 486);
  create_road("lincoln", "wichita", 277);
  create_road("lincoln", "omaha", 58);
  create_road("littleRock", "memphis", 137);
  create_road("littleRock", "tulsa", 276);
  create_road("losAngeles", "sanDiego", 124);
  create_road("losAngeles", "sanLuisObispo", 182);
  create_road("medford", "redding", 150);
  create_road("memphis", "nashville", 210);
  create_road("miami", "westPalmBeach", 67);
  create_road("midland", "toledo", 82);
  create_road("minneapolis", "winnipeg", 463);
  create_road("modesto", "stockton", 29);
  create_road("montreal", "ottawa", 132);
  create_road("newHaven", "providence", 110);
  create_road("newHaven", "stamford", 92);
  create_road("newOrleans", "pensacola", 268);
  create_road("newYork", "philadelphia", 101);
  create_road("norfolk", "richmond", 92);
  create_road("norfolk", "raleigh", 174);
  create_road("oakland", "sanFrancisco", 8);
  create_road("oakland", "sanJose", 42);
  create_road("oklahomaCity", "tulsa", 105);
  create_road("orlando", "westPalmBeach", 168);
  create_road("orlando", "tampa", 84);
  create_road("ottawa", "toronto", 269);
  create_road("pensacola", "tallahassee", 120);
  create_road("philadelphia", "pittsburgh", 319);
  create_road("philadelphia", "newYork", 101);
  create_road("philadelphia", "uk1", 3548);
  create_road("philadelphia", "uk2", 3548);
  create_road("phoenix", "tucson", 117);
  create_road("phoenix", "yuma", 178);
  create_road("pointReyes", "redding", 215);
  create_road("pointReyes", "sacramento", 115);
  create_road("portland", "seattle", 174);
  create_road("portland", "salem", 47);
  create_road("reno", "saltLakeCity", 520);
  create_road("reno", "sacramento", 133);
  create_road("richmond", "washington", 105);
  create_road("sacramento", "sanFrancisco", 95);
  create_road("sacramento", "stockton", 51);
  create_road("salinas", "sanJose", 31);
  create_road("salinas", "sanLuisObispo", 137);
  create_road("sanDiego", "yuma", 172);
  create_road("saultSteMarie", "thunderBay", 442);
  create_road("saultSteMarie", "toronto", 436);
  create_road("seattle", "vancouver", 115);
  create_road("thunderBay", "winnipeg", 440);
}
