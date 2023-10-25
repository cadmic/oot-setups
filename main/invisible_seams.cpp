#include <map>
#include <string>

#include "collision.hpp"
#include "collision_data.hpp"

std::map<std::string, CollisionHeader*> scenes = {
    {"Forest Temple", &Bmori1_sceneCollisionHeader_014054},
    {"Fire Temple", &FIRE_bs_sceneCollisionHeader_002BCC},
    {"Shadow Temple", &HAKAdan_sceneCollisionHeader_016394},
    {"Bottom of the Well", &HAKAdanCH_sceneCollisionHeader_00A558},
    {"Bongo Bongo", &HAKAdan_bs_sceneCollisionHeader_00134C},
    {"Fire Temple", &HIDAN_sceneCollisionHeader_01895C},
    {"Water Temple", &MIZUsin_sceneCollisionHeader_013C04},
    {"Morpha", &MIZUsin_bs_sceneCollisionHeader_001A34},
    {"Inside Jabu-Jabu's Belly", &bdan_sceneCollisionHeader_013074},
    {"Barinade", &bdan_boss_sceneCollisionHeader_000E14},
    {"Dodongo's Cavern", &ddan_sceneCollisionHeader_011D40},
    {"King Dodongo", &ddan_boss_sceneCollisionHeader_000E20},
    {"Ganon's Tower", &ganon_sceneCollisionHeader_00E7A0},
    {"Ganondorf", &ganon_boss_sceneCollisionHeader_001520},
    {"Ganon", &ganon_demo_sceneCollisionHeader_001AA0},
    {"Tower Collapse Exterior", &ganon_final_sceneCollisionHeader_002354},
    {"Tower Collapse Interior", &ganon_sonogo_sceneCollisionHeader_0062CC},
    {"Outside Ganon's Castle", &ganon_tou_sceneCollisionHeader_002610},
    {"Inside Ganon's Castle", &ganontika_sceneCollisionHeader_019EAC},
    {"Tower Collapse Interior Exit",
     &ganontikasonogo_sceneCollisionHeader_002ACC},
    {"Thieves' Hideout", &gerudoway_sceneCollisionHeader_0074EC},
    {"Ice Cavern", &ice_doukutu_sceneCollisionHeader_00F668},
    {"Twinrova", &jyasinboss_sceneCollisionHeader_002B80},
    {"Spirit Temple", &jyasinzou_sceneCollisionHeader_01680C},
    {"Gerudo Training Ground", &men_sceneCollisionHeader_00F690},
    {"Phantom Ganon", &moribossroom_sceneCollisionHeader_000B1C},
    {"Inside the Deku Tree", &ydan_sceneCollisionHeader_00B618},
    {"Queen Gohma", &ydan_boss_sceneCollisionHeader_000CFC},
    {"Bombchu Bowling Alley", &bowling_sceneCollisionHeader_001A74},
    {"Great Fairy Fountain (Upgrades)",
     &daiyousei_izumi_sceneCollisionHeader_0043A4},
    {"Hyrule Castle Guards (Day)", &hairal_niwa_sceneCollisionHeader_0030B0},
    // {"Unused", &hairal_niwa2_sceneCollisionHeader_002CD8},
    {"Hyrule Castle Guards (Night)",
     &hairal_niwa_n_sceneCollisionHeader_0010C4},
    {"Dampe's Grave and Windmill", &hakasitarelay_sceneCollisionHeader_00C04C},
    {"Dampe's Hut", &hut_sceneCollisionHeader_0004DC},
    {"Lakeside Laboratory", &hylia_labo_sceneCollisionHeader_00105C},
    {"Back Alley House (Dog Lady)", &impa_sceneCollisionHeader_000CE0},
    {"Carpenter Boss's House", &kakariko_sceneCollisionHeader_000E68},
    {"Chamber of the Sages", &kenjyanoma_sceneCollisionHeader_00359C},
    {"Know-It-All Brothers' House", &kokiri_home_sceneCollisionHeader_000C8C},
    {"House of Twins", &kokiri_home3_sceneCollisionHeader_001774},
    {"Mido's House", &kokiri_home4_sceneCollisionHeader_001A84},
    {"Saria's House", &kokiri_home5_sceneCollisionHeader_0013DC},
    {"Impa's House", &labo_sceneCollisionHeader_000EC4},
    {"Link's House", &link_home_sceneCollisionHeader_000E4C},
    {"Granny's Potion Shop", &mahouya_sceneCollisionHeader_0009F4},
    {"Stable", &malon_stable_sceneCollisionHeader_000644},
    {"Guard House (Lots of Pots and Big Poe Collector)",
     &miharigoya_sceneCollisionHeader_000B28},
    {"Hyrule Castle Courtyard", &nakaniwa_sceneCollisionHeader_001BC8},
    {"Shooting Gallery", &syatekijyou_sceneCollisionHeader_001420},
    {"Treasure Chest Game", &takaraya_sceneCollisionHeader_005178},
    {"Carpenters' Tent", &tent_sceneCollisionHeader_00064C},
    {"Temple of Time", &tokinoma_sceneCollisionHeader_0032F8},
    {"Fairy Fountain", &yousei_izumi_tate_sceneCollisionHeader_001FDC},
    {"Great Fairy Fountain (Spells)",
     &yousei_izumi_yoko_sceneCollisionHeader_0039A8},
    {"Market Entrance Ruins", &enrui_sceneCollisionHeader_0003B4},
    {"Market Entrance (Night)", &entra_n_sceneCollisionHeader_0003F8},
    {"Redead Grave (Sun's Song HP)", &hakaana_sceneCollisionHeader_000A60},
    {"Fairy Fountain Grave (Hylian Shield)",
     &hakaana2_sceneCollisionHeader_003058},
    {"Royal Family's Tomb", &hakaana_ouke_sceneCollisionHeader_002250},
    {"Cutscene Map", &hiral_demo_sceneCollisionHeader_003548},
    {"Back Alley House (Man in Green)", &kakariko3_sceneCollisionHeader_000808},
    {"Grottos", &kakusiana_sceneCollisionHeader_00B7F0},
    {"House of Skulltula", &kinsuta_sceneCollisionHeader_0015E4},
    {"Back Alley (Day)", &market_alley_sceneCollisionHeader_001218},
    {"Back Alley (Night)", &market_alley_n_sceneCollisionHeader_0012C0},
    {"Market (Day)", &market_day_sceneCollisionHeader_002640},
    {"Market (Night)", &market_night_sceneCollisionHeader_0025F8},
    {"Market Ruins", &market_ruins_sceneCollisionHeader_0015F8},
    {"Temple of Time Exterior (Day)", &shrine_sceneCollisionHeader_0014AC},
    {"Temple of Time Exterior (Night)", &shrine_n_sceneCollisionHeader_0014D4},
    {"Temple of Time Exterior Ruins", &shrine_r_sceneCollisionHeader_00145C},
    {"Fishing Pond", &turibori_sceneCollisionHeader_001CAC},
    {"Market Entrance (Day)", &entra_sceneCollisionHeader_0003B4},
    {"Lon Lon Ranch Tower", &souko_sceneCollisionHeader_0043E0},
    {"Hyrule Field", &spot00_sceneCollisionHeader_008464},
    {"Kakariko Village", &spot01_sceneCollisionHeader_004A1C},
    {"Graveyard", &spot02_sceneCollisionHeader_003C54},
    {"Zora's River", &spot03_sceneCollisionHeader_006580},
    {"Kokiri Forest", &spot04_sceneCollisionHeader_008918},
    {"Sacred Forest Meadow", &spot05_sceneCollisionHeader_003F4C},
    {"Lake Hylia", &spot06_sceneCollisionHeader_0055AC},
    {"Zora's Domain", &spot07_sceneCollisionHeader_003824},
    {"Zora's Fountain", &spot08_sceneCollisionHeader_002CE0},
    {"Gerudo Valley", &spot09_sceneCollisionHeader_002128},
    {"Lost Woods", &spot10_sceneCollisionHeader_00AC98},
    {"Desert Colossus", &spot11_sceneCollisionHeader_004EE4},
    {"Gerudo's Fortress", &spot12_sceneCollisionHeader_005030},
    {"Haunted Wasteland", &spot13_sceneCollisionHeader_003A00},
    {"Hyrule Castle", &spot15_sceneCollisionHeader_003CE8},
    {"Death Mountain Trail", &spot16_sceneCollisionHeader_003D10},
    {"Death Mountain Crater", &spot17_sceneCollisionHeader_0045A4},
    {"Goron City", &spot18_sceneCollisionHeader_0059AC},
    {"Lon Lon Ranch", &spot20_sceneCollisionHeader_002948},
    {"Market Potion Shop", &alley_shop_sceneCollisionHeader_000584},
    {"Kakariko Potion Shop", &drag_sceneCollisionHeader_0003C0},
    {"Happy Mask Shop", &face_shop_sceneCollisionHeader_000338},
    {"Goron Shop", &golon_sceneCollisionHeader_000368},
    {"Kokiri Shop", &kokiri_shop_sceneCollisionHeader_000950},
    {"Bombchu Shop", &night_shop_sceneCollisionHeader_000644},
    {"Bazaar", &shop1_sceneCollisionHeader_0002B8},
    {"Zora Shop", &zoora_sceneCollisionHeader_000360},
};

f32 intersect(f32 x, f32 z, Vec3f v, Vec3f normal, f32 dist) {
  if (SQ(v.z - z) + SQ(v.x - x) < SQ(1.0f)) {
    return (-normal.x * x - normal.z * z - dist) / normal.y;
  } else {
    return BGCHECK_Y_MIN;
  }
}

f32 step = 0.001f;

Vec3f highestPointApprox(Vec3f v, Vec3f normal, f32 dist) {
  Vec3f best;
  f32 ymax = BGCHECK_Y_MIN;
  for (f32 x = v.x - 1.0f; x <= v.x + 1.0f; x += step) {
    for (f32 z = v.z - 1.0f; z <= v.z + 1.0f; z += step) {
      f32 yIntersect = intersect(x, z, v, normal, dist);
      if (yIntersect > ymax) {
        ymax = yIntersect;
        best = {x, yIntersect, z};
      }
    }
  }
  return best;
}

Vec3f highestPoint(Vec3f v, Vec3f normal, f32 dist) {
  Vec3f best;
  f32 ymax = BGCHECK_Y_MIN;

  Vec3f approx = highestPointApprox(v, normal, dist);

  u32 xstart =
      std::min(floatToInt(approx.x - step), floatToInt(approx.x + step));
  u32 xend = std::max(floatToInt(approx.x - step), floatToInt(approx.x + step));
  u32 zstart =
      std::min(floatToInt(approx.z - step), floatToInt(approx.z + step));
  u32 zend = std::max(floatToInt(approx.z - step), floatToInt(approx.z + step));
  for (int xi = xstart; xi <= xend; xi++) {
    for (int zi = zstart; zi <= zend; zi++) {
      f32 x = intToFloat(xi);
      f32 z = intToFloat(zi);
      if (SQ(v.z - z) + SQ(v.x - x) < SQ(1.0f)) {
        f32 yIntersect = (-normal.x * x - normal.z * z - dist) / normal.y;
        if (yIntersect > ymax) {
          ymax = yIntersect;
          best = {x, yIntersect, z};
        }
      }
    }
  }
  return best;
}

void findInvisibleSeams() {
  printf("height,scene,poly index,vx,vy,vz,x,y,z,x (hex),y (hex),z (hex)\n");
  for (const auto& entry : scenes) {
    const std::string& name = entry.first;
    CollisionHeader* colHeader = entry.second;
    fprintf(stderr, "%s ... \r", name.c_str());

    for (int i = 0; i < colHeader->numPolys; i++) {
      CollisionPoly* poly = &colHeader->polys[i];

      Vec3f v[3];
      CollisionPoly_GetVertices(poly, colHeader->vertices, v);
      Vec3f normal = CollisionPoly_GetNormalF(poly);
      f32 dist = (s16)poly->dist;

      if (normal.y < 0.008f || normal.y > 0.1f) {
        continue;
      }

      f32 ymax = std::max(std::max(v[0].y, v[1].y), v[2].y);
      for (int j = 0; j < 3; j++) {
        if (v[j].y == ymax) {
          Vec3f seam = highestPoint(v[j], normal, dist);
          printf("%.9g,%s,0x%x,%.0f,%.0f,%.0f,%.9g,%.9g,%.9g,%08x,%08x,%08x\n",
                 seam.y - ymax, name.c_str(), i, v[j].x, v[j].y, v[j].z, seam.x,
                 seam.y, seam.z, floatToInt(seam.x), floatToInt(seam.y),
                 floatToInt(seam.z));
          fflush(stdout);
        }
      }
    }
  }
}

int main(int argc, char* argv[]) {
  findInvisibleSeams();

  return 0;
}
