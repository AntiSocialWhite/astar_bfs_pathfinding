#include <iostream>
#include <string>
#include <algorithm>
#include <chrono>
#include <fstream>
#include <queue>

using namespace std;

#include "EngineV2.h"

class PathFinding : public olcConsoleGameEngine
{
public:
	PathFinding()
	{
		m_sAppName = L"Path Finding - BFS & A*";
	}

private:

	struct s_pozicie
	{
		bool prekazka = false;
		bool navstivene = false;
		float_t dist_to_koniec;
		float_t dist_to_aktual;
		int32_t x;
		int32_t y;
		vector<s_pozicie*> susedia;
		s_pozicie* parent;
	};

	s_pozicie* nodes = nullptr;
	int32_t map_sirka = 16; // default dame na 16
	int32_t map_vyska = 16;

	s_pozicie* start = nullptr;
	s_pozicie* koniec = nullptr;

	int32_t mode = 1;
	bool preset = false;
	int32_t navstivene_pozicie = 0;
	std::chrono::microseconds cas;
	int32_t size_of_map = 1;
	int32_t velkost = 9; // default dame na 9
	int32_t okraj = 2;

protected:

	void on_create() {
		for (int32_t x = 0; x < map_sirka; x++) {
			for (int32_t y = 0; y < map_vyska; y++)
			{
				nodes[y * map_sirka + x].x = x;
				nodes[y * map_sirka + x].y = y;
				nodes[y * map_sirka + x].prekazka = false;
				nodes[y * map_sirka + x].parent = nullptr;
				nodes[y * map_sirka + x].navstivene = false;
			}
		}
	}

	void read_file() {
		std::ifstream myfile("preset" + std::to_string(size_of_map) + ".txt");		

		if (myfile.is_open()) {
			string tp;
			int32_t line = 0;
			while (getline(myfile, tp)) { 
				for (int32_t i = 0; i < map_sirka; i++) {
					if(tp.at(i) == '*')
						nodes[i * map_sirka + line].prekazka = true;
					else if(tp.at(i) == 'S')
						start = &nodes[i * map_sirka + line];
					else if (tp.at(i) == 'K')
						koniec = &nodes[i * map_sirka + line];
					else
						nodes[i * map_sirka + line].prekazka = false;
				}
				line++;
			}
			myfile.close();
		}
	}

	void write_bench(std::string mode, std::string time, std::string nodes, std::string preset) {
		std::fstream myfile;
		myfile.open("bench.txt", fstream::out);
		if (myfile.is_open()) {			
			myfile.write(mode.c_str(), mode.length());
			myfile.write(time.c_str(), time.length());
			myfile.write(nodes.c_str(), nodes.length());
			myfile.write(preset.c_str(), preset.length());
			myfile.close();
		}
	}

	void after_solve() {
		std::string mode_s_add;

		switch (mode) {
			case 1: mode_s_add = "A*"; break;
			case 2: mode_s_add = "BFS"; break;
			case 3: mode_s_add = "DIJKSTRA"; break;
			case 4: mode_s_add = "CUSTOM"; break;
		}

		std::string mode_s = "Mode: " + mode_s_add + "\n";
		std::string preset_s = "Preset: " + std::to_string(preset) + "\n";
		std::string nodes = "Pocet: " + std::to_string(navstivene_pozicie) + "\n";
		std::string time = "Mikrosekund: " + std::to_string(cas.count()) + "\n";
		write_bench(mode_s, time, nodes, preset_s);
	}

	void on_solve() {
		for (int32_t x = 0; x < map_sirka; x++) {
			for (int32_t y = 0; y < map_vyska; y++)
			{
				nodes[y * map_sirka + x].navstivene = false;
				nodes[y * map_sirka + x].dist_to_koniec = INFINITY;
				nodes[y * map_sirka + x].dist_to_aktual = INFINITY;
			}
		}
	}

	virtual bool OnUserCreate()
	{
		nodes = new s_pozicie[map_sirka * map_vyska];
		on_create();

		for (int32_t x = 0; x < map_sirka; x++) {
			for (int32_t y = 0; y < map_vyska; y++) // nastavenie susedov.
			{
				if (y > 0) { nodes[y * map_sirka + x].susedia.push_back(&nodes[(y - 1) * map_sirka + (x + 0)]); }
				if (y < map_vyska - 1) { nodes[y * map_sirka + x].susedia.push_back(&nodes[(y + 1) * map_sirka + (x + 0)]); }
				if (x > 0) { nodes[y * map_sirka + x].susedia.push_back(&nodes[(y + 0) * map_sirka + (x - 1)]); }
				if (x < map_sirka - 1) { nodes[y * map_sirka + x].susedia.push_back(&nodes[(y + 0) * map_sirka + (x + 1)]); }
			}
		}

		navstivene_pozicie = 0;
		cas.zero();
		start = &nodes[(map_vyska / 2) * map_sirka + 1];
		koniec = &nodes[(map_vyska / 2) * map_sirka + map_sirka - 2];
		return true;
	}

	float Distance(s_pozicie* a, s_pozicie* b)
	{
		 return fabsf(sqrtf((a->x - b->x) * (a->x - b->x) + (a->y - b->y) * (a->y - b->y))); // Euklidova
	}

	void Set_vars(s_pozicie* aktualny, s_pozicie* sused) {
		float lower = aktualny->dist_to_aktual + Distance(aktualny, sused);

		if (lower < sused->dist_to_aktual)
		{
			sused->parent = aktualny;
			sused->dist_to_aktual = lower;
			sused->dist_to_koniec = sused->dist_to_aktual + Distance(sused, koniec);
		}
	}

	bool Solve_BFS() {
		// reset
		on_solve();

		auto start_time = chrono::high_resolution_clock::now();
		navstivene_pozicie = 0;
		cas.zero();
		s_pozicie* aktualny = start;
		list<s_pozicie*> netestovane;
		netestovane.push_back(start);

		while (aktualny != koniec) {

			if (netestovane.empty())
				break;
	
			aktualny = netestovane.front();
			aktualny->navstivene = true; // navštívená
			navstivene_pozicie++; // pre pocet navstivenych

			netestovane.pop_front(); // pozície sa zbavíme, už sme ju navštívili

			for (auto sused : aktualny->susedia)
			{
				if (sused->prekazka == 0 && !sused->navstivene) {
					netestovane.push_back(sused);

					sused->parent = aktualny; // nastavujeme iba parents, vždy to bude prvá najlepšia cesta nako¾ko sa prechádza takmer celý graf.
				}
			}
		}

		auto end_time = chrono::high_resolution_clock::now();
		cas = chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
		return true;
	}


	bool Solve_Dijkstra() {
		// reset
		on_solve();
		auto start_time = chrono::high_resolution_clock::now();
		navstivene_pozicie = 0;
		cas.zero();
		s_pozicie* aktualny = start;
		s_pozicie* childNode = nullptr;
		start->dist_to_aktual = 0.0f;

		list<s_pozicie*> netestovane; // tak isto ako pri A* si vytvoríme list s netestovanými pozíciami a pridáme štart
		netestovane.push_back(start);

		while (!netestovane.empty() && aktualny != koniec) // najst najkratsiu cestu
		{
			while (!netestovane.empty() && netestovane.front()->navstivene) 
				netestovane.pop_front();

			if (netestovane.empty()) // sanity
				break;

			aktualny = netestovane.front();
			aktualny->navstivene = true; // navštívená
			navstivene_pozicie++; // pre pocet navstivenych

			for (const auto& sused : aktualny->susedia)
			{
				if (!sused->navstivene && sused->prekazka == 0) {

					netestovane.push_back(sused);

					float lower = aktualny->dist_to_aktual + Distance(aktualny, sused);

					if (lower < sused->dist_to_aktual + Distance(sused, koniec)){ // ve¾mi podobný princíp ako pri A*

						sused->parent = aktualny;
						sused->dist_to_aktual = lower;
					}
				}
			}
		}

		auto end_time = chrono::high_resolution_clock::now();
		cas = chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
		return true;
	}

	bool Solve_DIY() {
		// reset
		on_solve();
		auto start_time = chrono::high_resolution_clock::now();
		navstivene_pozicie = 0;
		cas.zero();
		s_pozicie* aktualny = start;
		s_pozicie* childNode = nullptr;
		start->dist_to_aktual = 0.0f;
		start->dist_to_koniec = Distance(start, koniec);

		list<s_pozicie*> netestovane;
		netestovane.push_back(start);

		for (int32_t x = 0; x < map_sirka; x++) {
			for (int32_t y = 0; y < map_vyska; y++)
			{
				if (start != &nodes[y * map_sirka + x]) {
					nodes[y * map_sirka + x].dist_to_koniec = Distance(&nodes[y * map_sirka + x], koniec); // nastavenie všetkých vzdialeností podla ktorých potom testujeme
					nodes[y * map_sirka + x].dist_to_aktual = Distance(&nodes[y * map_sirka + x], start); // nastavenie všetkých vzdialeností podla ktorých potom testujeme
				}
			}
		}

		while (!netestovane.empty() && aktualny != koniec) // najst najkratsiu cestu
		{
			while (!netestovane.empty() && netestovane.front()->navstivene)
				netestovane.pop_front();

			if (netestovane.empty()) // sanity
				break;

			netestovane.sort([](const s_pozicie* lhs, const s_pozicie* rhs) { return lhs->dist_to_aktual < rhs->dist_to_aktual; }); // sortovanie
			netestovane.sort([](const s_pozicie* lhs, const s_pozicie* rhs) { return lhs->dist_to_koniec < rhs->dist_to_koniec; }); // sortovanie

			aktualny = netestovane.front();
			aktualny->navstivene = true; // navštívená
			navstivene_pozicie++; // pre pocet navstivenych

			for (auto sused : aktualny->susedia)
			{
				if (!sused->navstivene && sused->prekazka == 0) {

					netestovane.push_back(sused);

					sused->parent = aktualny;
				}
			}
		}

		auto end_time = chrono::high_resolution_clock::now();
		cas = chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
		return true;
	}

	bool Solve()
	{
		// reset
		on_solve();

		auto start_time = chrono::high_resolution_clock::now();
		navstivene_pozicie = 0;
		cas.zero();
		s_pozicie* aktualny = start;
		start->dist_to_aktual = 0.0f;
		start->dist_to_koniec = Distance(start, koniec);

		list<s_pozicie*> netestovane;
		netestovane.push_back(start);

		while (!netestovane.empty() && aktualny != koniec) // najst najkratsiu cestu
		{
			netestovane.sort([](const s_pozicie* lhs, const s_pozicie* rhs) { return lhs->dist_to_koniec < rhs->dist_to_koniec; }); // sortovanie

			while (!netestovane.empty() && netestovane.front()->navstivene) // nenavštivujme ju viac krát
				netestovane.pop_front();

			if (netestovane.empty())
				break;

			aktualny = netestovane.front();
			aktualny->navstivene = true; // navštívená
			navstivene_pozicie++; // pre pocet navstivenych

			for (auto sused : aktualny->susedia)
			{
				//ak sme ešte nenavštívili a nieje to prekážka, pridajme suseda.
				if (!sused->navstivene && sused->prekazka == 0) {
					netestovane.push_back(sused);

					Set_vars(aktualny, sused);
				}
			}
		}

		auto end_time = chrono::high_resolution_clock::now();
		cas = chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
		return true;
	}

	void restart_solve() {
		switch (mode) {
		case 1: if (Solve()) after_solve(); break;
		case 2: if (Solve_BFS()) after_solve(); break;
		case 3: if (Solve_Dijkstra()) after_solve(); break;
		case 4: if (Solve_DIY()) after_solve(); break;
		}
	}

	void check_inputs() {
		int vybrata_poz_x = m_poz_x / velkost;
		int vybrata_poz_y = m_poz_y / velkost;

		if (m_keys[VK_DELETE].bPressed)
		{
			preset = !preset; // switch

			if (preset) {
				OnUserCreate();
				read_file();
			}
			else
				OnUserCreate(); // forcnutie resetu celej mapy.

			restart_solve();
		}

		if (m_keys[VK_F11].bPressed)
		{
			size_of_map++;

			if (size_of_map > 3)
				size_of_map = 1;

			switch (size_of_map) {
			case 1: {map_sirka = 16; map_vyska = 16; velkost = 9; break; };
			case 2: {map_sirka = 17; map_vyska = 17; velkost = 7; break; };
			case 3: {map_sirka = 18; map_vyska = 18; velkost = 7; break; };
			}

			if (preset) {
				OnUserCreate();
				read_file();
			}
			else
				OnUserCreate(); // forcnutie resetu celej mapy.

			restart_solve();
		}

		if (m_keys[VK_INSERT].bPressed)
		{
			mode++;

			if (mode > 4)
				mode = 1;

			restart_solve();
		}

		if (m_mouse[0].bPressed) // optimalizacia, runujemme solving iba po kliku myšky, predpokladáme že nejaký node alebo nieèo bolo zmenené
		{
			if (vybrata_poz_x >= 0 && vybrata_poz_x < map_sirka) {
				if (vybrata_poz_y >= 0 && vybrata_poz_y < map_vyska)
				{
					if (m_keys[VK_SHIFT].bHeld)
						start = &nodes[vybrata_poz_y * map_sirka + vybrata_poz_x];
					else if (m_keys[VK_CONTROL].bHeld)
						koniec = &nodes[vybrata_poz_y * map_sirka + vybrata_poz_x];
					else
						nodes[vybrata_poz_y * map_sirka + vybrata_poz_x].prekazka = !nodes[vybrata_poz_y * map_sirka + vybrata_poz_x].prekazka;

					restart_solve();
				}
			}
		}
	}

	void draw() {
		Fill(0, 0, ScreenWidth(), ScreenHeight(), L' ');

		for (int x = 0; x < map_sirka; x++) {
			for (int y = 0; y < map_vyska; y++)
			{
				for (auto n : nodes[y * map_sirka + x].susedia)
				{
					DrawLine(x * velkost + velkost / 2, y * velkost + velkost / 2, n->x * velkost + velkost / 2, n->y * velkost + velkost / 2, PIXEL_SOLID, FG_DARK_MAGENTA);
				}
			}
		}

		for (int x = 0; x < map_sirka; x++) {
			for (int y = 0; y < map_vyska; y++)
			{
				Fill(x * velkost + okraj, y * velkost + okraj, (x + 1) * velkost - okraj, (y + 1) * velkost - okraj, PIXEL_HALF, nodes[y * map_sirka + x].prekazka ? FG_DARK_RED : FG_DARK_BLUE);
				if (nodes[y * map_sirka + x].navstivene) { Fill(x * velkost + okraj, y * velkost + okraj, (x + 1) * velkost - okraj, (y + 1) * velkost - okraj, PIXEL_SOLID, FG_BLUE); }
				if (&nodes[y * map_sirka + x] == start) { Fill(x * velkost + okraj, y * velkost + okraj, (x + 1) * velkost - okraj, (y + 1) * velkost - okraj, PIXEL_SOLID, FG_GREEN); }
				if (&nodes[y * map_sirka + x] == koniec) { Fill(x * velkost + okraj, y * velkost + okraj, (x + 1) * velkost - okraj, (y + 1) * velkost - okraj, PIXEL_SOLID, FG_RED); }
			}
		}

		if (koniec != nullptr)
		{
			s_pozicie* p = koniec;
			while (p->parent != nullptr)
			{
				DrawLine(p->x * velkost + velkost / 2, p->y * velkost + velkost / 2, p->parent->x * velkost + velkost / 2, p->parent->y * velkost + velkost / 2, PIXEL_SOLID, FG_CYAN);
				p = p->parent;
			}
		}
	}

	void draw_menu() {
		DrawString(1, 145, L"Pre zmenu modu stlacte INSERT.");
		DrawString(1, 147, L"Pre zmenu velkosti mapy stlacte F11.");
		preset ? DrawString(1, 149, L"Preset aktivny [DELETE].") : DrawString(1, 149, L"Preset neaktivny [DELETE].");
		std::wstring pocet = L"Pocet:" + std::to_wstring(navstivene_pozicie);
		std::wstring cas_a = L"Mikrosekund: " + std::to_wstring(cas.count());

		switch (mode) {
			case 1: DrawString(130, 145, L"Mode: A*"); break;
			case 2: DrawString(130, 145, L"Mode: BFS"); break; 
			case 3: DrawString(130, 145, L"Mode: Dijkstra"); break;
			case 4: DrawString(130, 145, L"Mode: Custom"); break;
		}

		DrawString(130, 147, pocet);
		DrawString(130, 149, cas_a);
	}

	virtual bool OnUserUpdate(float fElapsedTime)
	{
		check_inputs(); // skontrolujeme èi sa nieèo nezmenilo
		draw(); // vykreslíme
		draw_menu(); // vykreslíme menu
		return true;
	}

};

int main()
{
	PathFinding PF;
	PF.ConstructConsole(160, 160, 6, 6);
	PF.Start();
	return 0;
}