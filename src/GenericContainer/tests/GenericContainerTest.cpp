/**
 * @file GenericContainerTest.cpp
 * @authors Stefano Dafarra
 * @copyright 2019 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

// Catch2
#include <catch2/catch_test_macros.hpp>
#include <iDynTree/VectorDynSize.h>
#include <iDynTree/VectorFixSize.h>
#include <iDynTree/TestUtils.h>
#include <iDynTree/Transform.h>
#include <memory>
#include <vector>
#include <string>
#include <Eigen/Core>

#include <BipedalLocomotion/GenericContainer/Vector.h>

using namespace BipedalLocomotion;

void foo(GenericContainer::Vector<double>::Ref test)
{
    test.data();
    return;
}

void fooConst(const GenericContainer::Vector<const double>::Ref test)
{
    test.data();
    return;
}

TEST_CASE("GenericContainer::Vector")
{
    SECTION("Constructible")
    {
        REQUIRE(GenericContainer::is_vector_constructible<iDynTree::VectorDynSize>::value);
        REQUIRE(GenericContainer::is_vector_constructible<iDynTree::VectorFixSize<3>>::value);
        REQUIRE(GenericContainer::is_vector_constructible<const iDynTree::VectorDynSize>::value);
        REQUIRE(GenericContainer::is_vector_constructible<const iDynTree::VectorFixSize<3>>::value);
        REQUIRE(GenericContainer::is_vector_constructible<std::vector<double>>::value);
        REQUIRE(GenericContainer::is_vector_constructible<std::vector<iDynTree::Transform>>::value);
        REQUIRE(GenericContainer::is_vector_constructible<const std::vector<double>>::value);
        REQUIRE(GenericContainer::is_vector_constructible<std::vector<int>>::value);
        REQUIRE(GenericContainer::is_vector_constructible<const std::vector<int>>::value);
        REQUIRE(GenericContainer::is_vector_constructible<std::vector<std::string>>::value);
        REQUIRE(GenericContainer::is_vector_constructible<double[5]>::value);
        REQUIRE(GenericContainer::is_vector_constructible<const double[5]>::value);
        REQUIRE(GenericContainer::is_vector_constructible<std::string>::value);
        REQUIRE(GenericContainer::is_vector_constructible<Eigen::VectorXd>::value);
        REQUIRE_FALSE(GenericContainer::is_vector_constructible<std::vector<bool>>::value);
        REQUIRE_FALSE(GenericContainer::is_vector_constructible<double>::value);
        REQUIRE_FALSE(GenericContainer::is_vector_constructible<int>::value);
        REQUIRE_FALSE(GenericContainer::is_vector_constructible<char>::value);
        REQUIRE_FALSE(GenericContainer::is_vector_constructible<void>::value);
    }

    SECTION("Copy")
    {
        iDynTree::VectorDynSize vector(5);
        iDynTree::getRandomVector(vector);
        GenericContainer::Vector container(iDynTree::make_span(vector));

        std::vector<double> copiedIn;
        copiedIn.resize(5);
        GenericContainer::Vector<double> containerToBeCopied(copiedIn); // copied in is automatically casted to a span

        containerToBeCopied = container;

        for (long i = 0; i < container.size(); ++i)
        {
            REQUIRE(vector[i] == copiedIn[i]);
        }

        iDynTree::getRandomVector(vector);
        containerToBeCopied = vector; //vector is automatically turned into a span
        for (long i = 0; i < container.size(); ++i)
        {
            REQUIRE(vector[i] == copiedIn[i]);
        }
    }

    SECTION("Impossible to resize")
    {
        iDynTree::VectorDynSize vector(5);
        GenericContainer::Vector container = GenericContainer::make_vector(vector);
        REQUIRE_FALSE(container.resizeVector(2));

        iDynTree::VectorFixSize<3> fixedVector;
        GenericContainer::Vector container2 = GenericContainer::make_vector(fixedVector, GenericContainer::VectorResizeMode::Resizable);
        REQUIRE_FALSE(container2.resizeVector(2));
    }

    SECTION("Resize")
    {
        iDynTree::VectorDynSize vector;

        GenericContainer::Vector container = GenericContainer::make_vector(vector, GenericContainer::VectorResizeMode::Resizable);
        REQUIRE(container.resizeVector(5));
        REQUIRE(vector.size() == 5);

    }

    SECTION("Resize and copy")
    {
        iDynTree::VectorDynSize vector(5);
        iDynTree::getRandomVector(vector);
        GenericContainer::Vector container(iDynTree::make_span(vector));

        std::vector<double> copiedIn;

        GenericContainer::Vector containerToBeCopied = GenericContainer::make_vector(copiedIn,
                                                                                     GenericContainer::VectorResizeMode::Resizable);

        containerToBeCopied = container;

        for (long i = 0; i < container.size(); ++i)
        {
            REQUIRE(vector[i] == copiedIn[i]);
        }
    }

    SECTION("Set/get value")
    {
        iDynTree::VectorDynSize vector(1);
        vector[0] = 0.0;
        GenericContainer::Vector container = GenericContainer::make_vector(vector);
        container[0] = 1.0;

        REQUIRE(vector[0] == 1.0);
        REQUIRE(container[0] == 1.0);
    }

    SECTION("Create const")
    {
        iDynTree::VectorDynSize vector(5);
        iDynTree::getRandomVector(vector);

        const iDynTree::VectorDynSize constVector = vector;
        GenericContainer::Vector container = GenericContainer::make_vector(constVector,
                                                                           GenericContainer::VectorResizeMode::Resizable);
    }

    SECTION("Create from self")
    {
        iDynTree::VectorDynSize vector(5);
        iDynTree::getRandomVector(vector);
        GenericContainer::Vector container = GenericContainer::make_vector(vector,
                                                                           GenericContainer::VectorResizeMode::Resizable);

        GenericContainer::Vector inception = GenericContainer::make_vector(container, GenericContainer::VectorResizeMode::Resizable);
    }

    SECTION("Create from self custom resize")
    {
        iDynTree::VectorDynSize vector(5);
        iDynTree::getRandomVector(vector);
        GenericContainer::Vector container = GenericContainer::make_vector(vector,
                                                                           GenericContainer::VectorResizeMode::Resizable);

        using resize_function = GenericContainer::Vector<double>::resize_function_type;
        using index_type = GenericContainer::Vector<double>::index_type;

        GenericContainer::Vector<double>* inputPtr = &container;
        resize_function resizeLambda =
            [inputPtr](index_type newSize) -> iDynTree::Span<double>
        {
            inputPtr->resizeVector(newSize);
            std::cerr << "I am resizing!" << std::endl;
            return iDynTree::make_span(*inputPtr);
        };

        GenericContainer::Vector inception(iDynTree::make_span(container), resizeLambda);
        REQUIRE(inception.resizeVector(6));
        REQUIRE(vector.size() == 6);
    }

    SECTION("String")
    {
        std::string test = "Test";
        GenericContainer::Vector container = GenericContainer::make_vector(test, GenericContainer::VectorResizeMode::Resizable);

        container.resize(3);

        REQUIRE(test == "Tes");
    }

    SECTION("Create pointer")
    {
        iDynTree::VectorDynSize vector(5);
        iDynTree::getRandomVector(vector);
        GenericContainer::Vector_ptr<double> container_ptr = GenericContainer::make_vector_ptr(vector,
                                                                                       GenericContainer::VectorResizeMode::Resizable);
        REQUIRE(container_ptr);

        const iDynTree::VectorDynSize constVector = vector;
        GenericContainer::Vector_ptr<const double> const_container_ptr = GenericContainer::make_vector_ptr(constVector);
        REQUIRE(const_container_ptr);

        std::vector<double> copiedIn;
        copiedIn.resize(5);
        container_ptr = GenericContainer::make_vector_ptr(iDynTree::make_span(copiedIn));
        REQUIRE(container_ptr);


        GenericContainer::Vector container = GenericContainer::make_vector(vector,
                                                                           GenericContainer::VectorResizeMode::Resizable);

        using resize_function = GenericContainer::Vector<double>::resize_function_type;
        using index_type = GenericContainer::Vector<double>::index_type;

        GenericContainer::Vector<double>* inputPtr = &container;
        resize_function resizeLambda =
            [inputPtr](index_type newSize) -> iDynTree::Span<double>
        {
            inputPtr->resizeVector(newSize);
            std::cerr << "I am resizing again!" << std::endl;
            return iDynTree::make_span(*inputPtr);
        };

        container_ptr = GenericContainer::make_vector_ptr(iDynTree::make_span(container), resizeLambda);
        REQUIRE(container_ptr->resizeVector(6));
        REQUIRE(vector.size() == 6);

    }


    SECTION("at_call")
    {
        int arr[4] = {1, 2, 3, 4};

        {
            GenericContainer::Vector s = GenericContainer::make_vector(arr);
            REQUIRE(s.at(0) == 1);
        }

        {
            int arr2d[2] = {1, 6};
            GenericContainer::Vector s = GenericContainer::make_vector(arr2d);
            REQUIRE(s.at(0) == 1);
            REQUIRE(s.at(1) == 6);
        }
    }

    SECTION("operator_function_call")
    {
        int arr[4] = {1, 2, 3, 4};

        {
            GenericContainer::Vector s = GenericContainer::make_vector(arr);
            REQUIRE(s(0) == 1);
        }

        {
            int arr2d[2] = {1, 6};
            GenericContainer::Vector s = GenericContainer::make_vector(arr2d);
            REQUIRE(s(0) == 1);
            REQUIRE(s(1) == 6);
        }
    }

    SECTION("iterator_default_init")
    {
        GenericContainer::Vector<int>::iterator it1;
        GenericContainer::Vector<int>::iterator it2;
        bool ok = (it1 == it2);
        REQUIRE(ok);
    }

    SECTION("const_iterator_default_init")
    {
        GenericContainer::Vector<int>::const_iterator it1;
        GenericContainer::Vector<int>::const_iterator it2;
        bool ok = (it1 == it2);
        REQUIRE(ok);
    }

    SECTION("iterator_conversions")
    {
        bool ok;

        GenericContainer::Vector<int>::iterator badIt;
        GenericContainer::Vector<int>::const_iterator badConstIt;
        ok = (badIt == badConstIt);
        REQUIRE(ok);

        int a[] = {1, 2, 3, 4};
        GenericContainer::Vector s = GenericContainer::make_vector(a);

        auto it = s.begin();
        auto cit = s.cbegin();

        ok = (it == cit);
        REQUIRE(ok);
        ok = (cit == it);
        REQUIRE(ok);

        GenericContainer::Vector<int>::const_iterator cit2 = it;
        ok = (cit2 == cit);
        REQUIRE(ok);

        GenericContainer::Vector<int>::const_iterator cit3 = it + 4;
        ok = (cit3 == s.cend());
        REQUIRE(ok);
    }

    SECTION("iterator_comparisons")
    {
        int a[] = {1, 2, 3, 4};
        {
            GenericContainer::Vector s = GenericContainer::make_vector(a);
            GenericContainer::Vector<int>::iterator it = s.begin();
            auto it2 = it + 1;
            GenericContainer::Vector<int>::const_iterator cit = s.cbegin();
            bool ok;

            ok = (it == cit);
            REQUIRE(ok);
            ok = (cit == it);
            REQUIRE(ok);
            ok = (it == it);
            REQUIRE(ok);
            ok = (cit == cit);
            REQUIRE(ok);
            ok = (cit == s.begin());
            REQUIRE(ok);
            ok = (s.begin() == cit);
            REQUIRE(ok);
            ok = (s.cbegin() == cit);
            REQUIRE(ok);
            ok = (it == s.begin());
            REQUIRE(ok);
            ok = (s.begin() == it);
            REQUIRE(ok);

            ok = (it != it2);
            REQUIRE(ok);
            ok = (it2 != it);
            REQUIRE(ok);
            ok = (it != s.end());
            REQUIRE(ok);
            ok = (it2 != s.end());
            REQUIRE(ok);
            ok = (s.end() != it);
            REQUIRE(ok);
            ok = (it2 != cit);
            REQUIRE(ok);
            ok = (cit != it2);
            REQUIRE(ok);

            ok = (it < it2);
            REQUIRE(ok);
            ok = (it <= it2);
            REQUIRE(ok);
            ok = (it2 <= s.end());
            REQUIRE(ok);
            ok = (it < s.end());
            REQUIRE(ok);
            ok = (it <= cit);
            REQUIRE(ok);
            ok = (cit <= it);
            REQUIRE(ok);
            ok = (cit < it2);
            REQUIRE(ok);
            ok = (cit <= it2);
            REQUIRE(ok);
            ok = (cit < s.end());
            REQUIRE(ok);
            ok = (cit <= s.end());
            REQUIRE(ok);

            ok = (it2 > it);
            REQUIRE(ok);
            ok = (it2 >= it);
            REQUIRE(ok);
            ok = (s.end() > it2);
            REQUIRE(ok);
            ok = (s.end() >= it2);
            REQUIRE(ok);
            ok = (it2 > cit);
            REQUIRE(ok);
            ok = (it2 >= cit);
            REQUIRE(ok);
        }
    }

    SECTION("begin_end")
    {
        {
            int a[] = {1, 2, 3, 4};
            GenericContainer::Vector s = GenericContainer::make_vector(a);

            GenericContainer::Vector<int>::iterator it = s.begin();
            GenericContainer::Vector<int>::iterator it2 = std::begin(s);
            bool ok;
            ok = (it == it2);
            REQUIRE(ok);

            it = s.end();
            it2 = std::end(s);
            ok = (it == it2);
            REQUIRE(ok);
        }

        {
            int a[] = {1, 2, 3, 4};
            GenericContainer::Vector s = GenericContainer::make_vector(a);

            auto it = s.begin();
            auto first = it;
            bool ok;

            ok = (it == first);
            REQUIRE(ok);
            ok = (*it == 1);
            REQUIRE(ok);

            auto beyond = s.end();
            ok = (it != beyond);
            REQUIRE(ok);

            ok = (beyond - first == 4);
            REQUIRE(ok);
            ok = (first - first == 0);
            REQUIRE(ok);
            ok = (beyond - beyond == 0);
            REQUIRE(ok);

            ++it;
            ok = (it - first == 1);
            REQUIRE(ok);
            ok = (*it == 2);
            REQUIRE(ok);
            *it = 22;
            ok = (*it == 22);
            REQUIRE(ok);
            ok = (beyond - it == 3);
            REQUIRE(ok);

            it = first;
            ok = (it == first);
            REQUIRE(ok);
            while (it != s.end()) {
                *it = 5;
                ++it;
            }

            ok = (it == beyond);
            REQUIRE(ok);
            ok = (it - beyond == 0);
            REQUIRE(ok);

            for (const auto& n : s) {
                ok = (n == 5);
                REQUIRE(ok);
            }
        }
    }

    SECTION("cbegin_cend")
    {
        {
            int a[] = {1, 2, 3, 4};
            GenericContainer::Vector s = GenericContainer::make_vector(a);

            GenericContainer::Vector<int>::const_iterator cit = s.cbegin();
            GenericContainer::Vector<int>::const_iterator cit2 = std::cbegin(s);
            bool ok;
            ok = (cit == cit2);
            REQUIRE(ok);

            cit = s.cend();
            cit2 = std::cend(s);
            ok = (cit == cit2);
            REQUIRE(ok);
        }

        {
            int a[] = {1, 2, 3, 4};
            GenericContainer::Vector s = GenericContainer::make_vector(a);

            auto it = s.cbegin();
            auto first = it;
            bool ok;
            ok = (it == first);
            REQUIRE(ok);
            ok = (*it == 1);
            REQUIRE(ok);

            auto beyond = s.cend();
            ok = (it != beyond);
            REQUIRE(ok);

            ok = (beyond - first == 4);
            REQUIRE(ok);
            ok = (first - first == 0);
            REQUIRE(ok);
            ok = (beyond - beyond == 0);
            REQUIRE(ok);

            ++it;
            ok = (it - first == 1);
            REQUIRE(ok);
            ok = (*it == 2);
            REQUIRE(ok);
            ok = (beyond - it == 3);
            REQUIRE(ok);

            int last = 0;
            it = first;
            ok = (it == first);
            REQUIRE(ok);
            while (it != s.cend()) {
                ok = (*it == last + 1);
                REQUIRE(ok);

                last = *it;
                ++it;
            }

            ok = (it == beyond);
            REQUIRE(ok);
            ok = (it - beyond == 0);
            REQUIRE(ok);
        }
    }

    SECTION("rbegin_rend")
    {
        {
            int a[] = {1, 2, 3, 4};
            GenericContainer::Vector s = GenericContainer::make_vector(a);

            auto it = s.rbegin();
            auto first = it;
            bool ok;
            ok = (it == first);
            REQUIRE(ok);
            ok = (*it == 4);
            REQUIRE(ok);

            auto beyond = s.rend();
            ok = (it != beyond);
            REQUIRE(ok);

            ok = (beyond - first == 4);
            REQUIRE(ok);
            ok = (first - first == 0);
            REQUIRE(ok);
            ok = (beyond - beyond == 0);
            REQUIRE(ok);

            ++it;
            ok = (it - first == 1);
            REQUIRE(ok);
            ok = (*it == 3);
            REQUIRE(ok);
            *it = 22;
            ok = (*it == 22);
            REQUIRE(ok);
            ok = (beyond - it == 3);
            REQUIRE(ok);

            it = first;
            ok = (it == first);
            REQUIRE(ok);
            while (it != s.rend()) {
                *it = 5;
                ++it;
            }

            ok = (it == beyond);
            REQUIRE(ok);
            ok = (it - beyond == 0);
            REQUIRE(ok);

            for (const auto& n : s) {
                ok = (n == 5);
                REQUIRE(ok);
            }
        }
    }

    SECTION("crbegin_crend")
    {
        {
            int a[] = {1, 2, 3, 4};
            GenericContainer::Vector s = GenericContainer::make_vector(a);

            auto it = s.crbegin();
            auto first = it;
            bool ok;
            ok = (it == first);
            REQUIRE(ok);
            ok = (*it == 4);
            REQUIRE(ok);

            auto beyond = s.crend();
            ok = (it != beyond);
            REQUIRE(ok);
            ok = (beyond - first == 4);
            REQUIRE(ok);
            ok = (first - first == 0);
            REQUIRE(ok);
            ok = (beyond - beyond == 0);
            REQUIRE(ok);

            ++it;
            ok = (it - first == 1);
            REQUIRE(ok);
            ok = (*it == 3);
            REQUIRE(ok);
            ok = (beyond - it == 3);
            REQUIRE(ok);

            it = first;
            ok = (it == first);
            REQUIRE(ok);
            int last = 5;
            while (it != s.crend()) {
                ok = (*it == last - 1);
                REQUIRE(ok);
                last = *it;

                ++it;
            }

            ok = (it == beyond);
            REQUIRE(ok);
            ok = (it - beyond == 0);
            REQUIRE(ok);
        }
    }

    SECTION("To eigen")
    {
        double a[] = {1, 2, 3, 4};
        iDynTree::VectorDynSize b(4);
        b(0) = 5;
        b(1) = 6;
        b(2) = 7;
        b(3) = 8;
        std::vector<double> c{6, 8, 10, 12};

        Eigen::VectorXd d = GenericContainer::to_eigen(a) + GenericContainer::to_eigen(b);

        REQUIRE(d.isApprox(GenericContainer::to_eigen(c)));
    }

    SECTION("Refs")
    {
        std::vector<int> vec(5);
        GenericContainer::Vector<int>::Ref stdRef(vec);
        const std::vector<int>& cvec = vec;
        GenericContainer::Vector<const int>::Ref stdConstRef(cvec);
        Eigen::Vector2d eigenVec;
        GenericContainer::Vector<double>::Ref eigenRef(eigenVec);
        iDynTree::VectorFixSize<3> idynFixVec;
        GenericContainer::Vector<double>::Ref idynFix(idynFixVec);
        iDynTree::VectorDynSize idynVec;
        GenericContainer::Vector<double>::Ref idyn(idynVec);
        const iDynTree::VectorDynSize& idynConstVec = idynVec;
        GenericContainer::Vector<const double>::Ref idynConst(idynConstVec);
    }

    SECTION("Generic input to function")
    {
        std::vector<double> vec(5);
        foo(vec);
        const std::vector<double>& cvec = vec;
        fooConst(cvec);
        Eigen::Vector2d eigenVec;
        foo(eigenVec);
        iDynTree::VectorFixSize<3> idynFixVec;
        foo(idynFixVec);
        iDynTree::VectorDynSize idynVec;
        foo(idynVec);
        fooConst(idynVec);
        foo(GenericContainer::make_vector(idynVec, GenericContainer::VectorResizeMode::Fixed));
    }

    SECTION("Copy of Refs")
    {
        iDynTree::VectorDynSize vector(5);
        iDynTree::getRandomVector(vector);
        GenericContainer::Vector<double>::Ref container(vector);

        std::vector<double> copiedIn;
        copiedIn.resize(5);
        GenericContainer::Vector<double> containerToBeCopied(copiedIn); // copied in is automatically casted to a span

        containerToBeCopied = container; //Ref = Vector

        for (long i = 0; i < container.size(); ++i)
        {
            REQUIRE(vector[i] == copiedIn[i]);
        }

        Eigen::VectorXd otherCopiedIn;
        GenericContainer::Vector<double>::Ref otherContainerToBeCopied(otherCopiedIn);

        otherContainerToBeCopied = container;

        for (long i = 0; i < container.size(); ++i)
        {
            REQUIRE(vector[i] == otherCopiedIn[i]);
        }
    }

}
