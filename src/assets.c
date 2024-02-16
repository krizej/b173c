#include "common.h"
#include "assets.h"
#include <SDL2/SDL.h>
#define STB_IMAGE_IMPLEMENTATION
#include "stb/stb_image.h"

struct asset_storage builtin_assets[ASSET_COUNT] = {0};
struct asset_storage loaded_assets[ASSET_COUNT] = {0};
bool is_asset_loaded[ASSET_COUNT] = {0};

#define null_path ""
#define basedir "assets"
const char *asset_paths[ASSET_COUNT] = {
	[ASSET_NULL] = null_path,
	[ASSET_TEXTURE_TERRAIN] = basedir"/terrain.png",
	[ASSET_TEXTURE_FONT_DEFAULT] = basedir"/font/default.png",
};

asset_text *asset_get_text(asset_name name)
{
	size_t len;
	void *data;
	asset_text *asset;

	/* check cache */
	if(is_asset_loaded[name]) {
		return (asset_text *) &loaded_assets[name];
	}

	/* try to overwrite the builtin asset */
	if(!(data = SDL_LoadFile(asset_paths[name], &len))) {
		/* failed - load builtin */
		is_asset_loaded[name] = true;
		loaded_assets[name] = builtin_assets[name];
		return (asset_text *) &loaded_assets[name];
	}

	/* got the file */
	is_asset_loaded[name] = true;
	asset = (asset_text *) &loaded_assets[name];
	asset->data = data;
	asset->length = len;

	return asset;
}

asset_image *asset_get_image(asset_name name)
{
	ubyte *data;
	asset_image *asset;

	asset = (asset_image *) &loaded_assets[name];

	/* check cache */
	if(is_asset_loaded[name])
		return asset;

	/* try to overwrite the builtin asset */
	if(!(data = stbi_load(asset_paths[name], &asset->width, &asset->height, &asset->channels, 4))) {
		/* failed - load builtin */
		*(struct asset_storage *)asset = builtin_assets[name];
		is_asset_loaded[name] = true;
		return asset;
	}

	/* got the file */
	is_asset_loaded[name] = true;
	asset->data = data;
	/* other fields got filled out by the stbi_load call */

	return asset;
}

