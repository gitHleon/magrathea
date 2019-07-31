/*
 * test.flatness.cc
 */
#include <string>
#include <xlnt/xlnt.hpp>
#include "flatness.h"

std::string get_file_extension(const std::string &fname)
{
    std::string::size_type dpos = fname.find_last_of( '.' );
    if (dpos == fname.npos)
        return std::string();
    else
        return fname.substr(dpos+1);
}

int read_xlsx_file(const char *fname, alglib::real_2d_array &CC)
{
    xlnt::workbook wb;
    try
    {
        wb.load(fname);
    }
    catch (xlnt::exception &e)
    {
        std::cout << "Excel file " << fname << " not found" << std::endl;
        return -1;
    }
    const xlnt::worksheet ws = wb.active_sheet();

    // Count number of rows
    std::vector< std::vector<double> > table;

    int first = 0;
    for (auto row : ws.rows(false))
    {
        if (first++ == 0)
            continue;

        std::vector<double> vrow;
        for (auto cell : row)
            vrow.push_back(cell.value<double>());

        table.push_back(vrow);
    }

    CC.setlength(table.size(), 3);
    int irow = 0;
    for (auto row : table)
    {
        int icol = 0;
        for ( auto val : row )
            CC[irow][icol++] = val;

        irow++;
    }
    return 0;
}

int main(int argc, char **argv)
{
    if (argv[1] == nullptr)
    {
        std::cout << "I need a data file" << std::endl;
        return 1;
    }

    /*
     * Check file type
     */
    std::string ifile(argv[1]);
    std::string ext = get_file_extension(ifile);

    alglib::real_2d_array CC;
    if (ext=="xlsx" || ext=="xls")
        read_xlsx_file(ifile.c_str(), CC);
    else
        alglib::read_csv(ifile.c_str(), '\t', 0, CC);

    // Compute flatness
    double F = flatness_lin(CC);
    std::cout << "Flatness: " << F << std::endl;
    return 0;
}

