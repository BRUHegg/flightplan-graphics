#include <fstream>
#include <string>
#include <assert.h>
#include <vector>
#include <map>


namespace byteutils
{
    const std::string BYTEMAP_FORMAT = ".byt";


    inline bool does_file_exist(std::string name)
	{
		std::ifstream file(name, std::ifstream::in);
		if (file.is_open())
		{
			file.close();
			return true;
		}
		file.close();
		return false;
	}


    class Bytemap
    {
    public:
        Bytemap(size_t w, size_t h)
        {
            width = w;
            height = h;
        }

        size_t get_width()
        {
            return width;
        }

        size_t get_height()
        {
            return height;
        }

        bool load(std::string f_path)
        {
            if(does_file_exist(f_path))
            {
                std::ifstream input(f_path, std::ios::binary);
                buf = std::vector<unsigned char>(std::istreambuf_iterator<char>(input), {});

                return true;
            }

            return false;
        }

        unsigned char get_at(size_t x, size_t y)
        {
            assert(x < width && y < height);
            return buf[y * width + x];
        }

    private:
        std::vector<unsigned char> buf;
        size_t width, height;
    };


    struct bytemap_manager_t
    {
        std::map<std::string, Bytemap*> data;


        bool add_bytemap(std::string path, std::string f_name, size_t w, size_t h)
        {
            Bytemap *tmp = new Bytemap(w, h);
            if(tmp->load(path+f_name+BYTEMAP_FORMAT))
            {
                data[f_name] = tmp;
                return true;
            }

            delete tmp;

            return false;
        }

        Bytemap* get_bytemap(std::string f_name)
        {
            if(data.find(f_name) != data.end())
            {
                return data[f_name];
            }

            return nullptr;
        }

        void destroy()
        {
            for(auto i: data)
                delete i.second;
        }
    };
};
