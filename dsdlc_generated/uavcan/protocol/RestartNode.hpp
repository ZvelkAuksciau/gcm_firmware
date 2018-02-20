/*
 * UAVCAN data structure definition for libuavcan.
 *
 * Autogenerated, do not edit.
 *
 * Source file: /home/valaitis/workspace/kmti/gimbal_ctrl/modules/libuavcan/dsdl/uavcan/protocol/5.RestartNode.uavcan
 */

#ifndef UAVCAN_PROTOCOL_RESTARTNODE_HPP_INCLUDED
#define UAVCAN_PROTOCOL_RESTARTNODE_HPP_INCLUDED

#include <uavcan/build_config.hpp>
#include <uavcan/node/global_data_type_registry.hpp>
#include <uavcan/marshal/types.hpp>

/******************************* Source text **********************************
#
# Restart the node.
#
# Some nodes may require restart before the new configuration will be applied.
#
# The request should be rejected if magic_number does not equal MAGIC_NUMBER.
#

uint40 MAGIC_NUMBER = 0xACCE551B1E
uint40 magic_number

---

bool ok
******************************************************************************/

/********************* DSDL signature source definition ***********************
uavcan.protocol.RestartNode
saturated uint40 magic_number
---
saturated bool ok
******************************************************************************/

#undef magic_number
#undef MAGIC_NUMBER
#undef ok

namespace uavcan
{
namespace protocol
{

struct UAVCAN_EXPORT RestartNode_
{
    template <int _tmpl>
    struct Request_
    {
        typedef const Request_<_tmpl>& ParameterType;
        typedef Request_<_tmpl>& ReferenceType;

        struct ConstantTypes
        {
            typedef ::uavcan::IntegerSpec< 40, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > MAGIC_NUMBER;
        };

        struct FieldTypes
        {
            typedef ::uavcan::IntegerSpec< 40, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > magic_number;
        };

        enum
        {
            MinBitLen
                = FieldTypes::magic_number::MinBitLen
        };

        enum
        {
            MaxBitLen
                = FieldTypes::magic_number::MaxBitLen
        };

        // Constants
        static const typename ::uavcan::StorageType< typename ConstantTypes::MAGIC_NUMBER >::Type MAGIC_NUMBER; // 0xACCE551B1E

        // Fields
        typename ::uavcan::StorageType< typename FieldTypes::magic_number >::Type magic_number;

        Request_()
            : magic_number()
        {
            ::uavcan::StaticAssert<_tmpl == 0>::check();  // Usage check

    #if UAVCAN_DEBUG
            /*
             * Cross-checking MaxBitLen provided by the DSDL compiler.
             * This check shall never be performed in user code because MaxBitLen value
             * actually depends on the nested types, thus it is not invariant.
             */
            ::uavcan::StaticAssert<40 == MaxBitLen>::check();
    #endif
        }

        bool operator==(ParameterType rhs) const;
        bool operator!=(ParameterType rhs) const { return !operator==(rhs); }

        /**
         * This comparison is based on @ref uavcan::areClose(), which ensures proper comparison of
         * floating point fields at any depth.
         */
        bool isClose(ParameterType rhs) const;

        static int encode(ParameterType self, ::uavcan::ScalarCodec& codec,
                          ::uavcan::TailArrayOptimizationMode tao_mode = ::uavcan::TailArrayOptEnabled);

        static int decode(ReferenceType self, ::uavcan::ScalarCodec& codec,
                          ::uavcan::TailArrayOptimizationMode tao_mode = ::uavcan::TailArrayOptEnabled);

    };

    template <int _tmpl>
    struct Response_
    {
        typedef const Response_<_tmpl>& ParameterType;
        typedef Response_<_tmpl>& ReferenceType;

        struct ConstantTypes
        {
        };

        struct FieldTypes
        {
            typedef ::uavcan::IntegerSpec< 1, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > ok;
        };

        enum
        {
            MinBitLen
                = FieldTypes::ok::MinBitLen
        };

        enum
        {
            MaxBitLen
                = FieldTypes::ok::MaxBitLen
        };

        // Constants

        // Fields
        typename ::uavcan::StorageType< typename FieldTypes::ok >::Type ok;

        Response_()
            : ok()
        {
            ::uavcan::StaticAssert<_tmpl == 0>::check();  // Usage check

    #if UAVCAN_DEBUG
            /*
             * Cross-checking MaxBitLen provided by the DSDL compiler.
             * This check shall never be performed in user code because MaxBitLen value
             * actually depends on the nested types, thus it is not invariant.
             */
            ::uavcan::StaticAssert<1 == MaxBitLen>::check();
    #endif
        }

        bool operator==(ParameterType rhs) const;
        bool operator!=(ParameterType rhs) const { return !operator==(rhs); }

        /**
         * This comparison is based on @ref uavcan::areClose(), which ensures proper comparison of
         * floating point fields at any depth.
         */
        bool isClose(ParameterType rhs) const;

        static int encode(ParameterType self, ::uavcan::ScalarCodec& codec,
                          ::uavcan::TailArrayOptimizationMode tao_mode = ::uavcan::TailArrayOptEnabled);

        static int decode(ReferenceType self, ::uavcan::ScalarCodec& codec,
                          ::uavcan::TailArrayOptimizationMode tao_mode = ::uavcan::TailArrayOptEnabled);

    };

    typedef Request_<0> Request;
    typedef Response_<0> Response;

    /*
     * Static type info
     */
    enum { DataTypeKind = ::uavcan::DataTypeKindService };
    enum { DefaultDataTypeID = 5 };

    static const char* getDataTypeFullName()
    {
        return "uavcan.protocol.RestartNode";
    }

    static void extendDataTypeSignature(::uavcan::DataTypeSignature& signature)
    {
        signature.extend(getDataTypeSignature());
    }

    static ::uavcan::DataTypeSignature getDataTypeSignature();

private:
    RestartNode_(); // Don't create objects of this type. Use Request/Response instead.
};

/*
 * Out of line struct method definitions
 */

template <int _tmpl>
bool RestartNode_::Request_<_tmpl>::operator==(ParameterType rhs) const
{
    return
        magic_number == rhs.magic_number;
}

template <int _tmpl>
bool RestartNode_::Request_<_tmpl>::isClose(ParameterType rhs) const
{
    return
        ::uavcan::areClose(magic_number, rhs.magic_number);
}

template <int _tmpl>
int RestartNode_::Request_<_tmpl>::encode(ParameterType self, ::uavcan::ScalarCodec& codec,
    ::uavcan::TailArrayOptimizationMode tao_mode)
{
    (void)self;
    (void)codec;
    (void)tao_mode;
    int res = 1;
    res = FieldTypes::magic_number::encode(self.magic_number, codec,  tao_mode);
    return res;
}

template <int _tmpl>
int RestartNode_::Request_<_tmpl>::decode(ReferenceType self, ::uavcan::ScalarCodec& codec,
    ::uavcan::TailArrayOptimizationMode tao_mode)
{
    (void)self;
    (void)codec;
    (void)tao_mode;
    int res = 1;
    res = FieldTypes::magic_number::decode(self.magic_number, codec,  tao_mode);
    return res;
}

template <int _tmpl>
bool RestartNode_::Response_<_tmpl>::operator==(ParameterType rhs) const
{
    return
        ok == rhs.ok;
}

template <int _tmpl>
bool RestartNode_::Response_<_tmpl>::isClose(ParameterType rhs) const
{
    return
        ::uavcan::areClose(ok, rhs.ok);
}

template <int _tmpl>
int RestartNode_::Response_<_tmpl>::encode(ParameterType self, ::uavcan::ScalarCodec& codec,
    ::uavcan::TailArrayOptimizationMode tao_mode)
{
    (void)self;
    (void)codec;
    (void)tao_mode;
    int res = 1;
    res = FieldTypes::ok::encode(self.ok, codec,  tao_mode);
    return res;
}

template <int _tmpl>
int RestartNode_::Response_<_tmpl>::decode(ReferenceType self, ::uavcan::ScalarCodec& codec,
    ::uavcan::TailArrayOptimizationMode tao_mode)
{
    (void)self;
    (void)codec;
    (void)tao_mode;
    int res = 1;
    res = FieldTypes::ok::decode(self.ok, codec,  tao_mode);
    return res;
}

/*
 * Out of line type method definitions
 */
inline ::uavcan::DataTypeSignature RestartNode_::getDataTypeSignature()
{
    ::uavcan::DataTypeSignature signature(0x569E05394A3017F0ULL);

    Request::FieldTypes::magic_number::extendDataTypeSignature(signature);

    Response::FieldTypes::ok::extendDataTypeSignature(signature);

    return signature;
}

/*
 * Out of line constant definitions
 */

template <int _tmpl>
const typename ::uavcan::StorageType< typename RestartNode_::Request_<_tmpl>::ConstantTypes::MAGIC_NUMBER >::Type
    RestartNode_::Request_<_tmpl>::MAGIC_NUMBER = 742196058910U; // 0xACCE551B1E

/*
 * Final typedef
 */
typedef RestartNode_ RestartNode;

namespace
{

const ::uavcan::DefaultDataTypeRegistrator< ::uavcan::protocol::RestartNode > _uavcan_gdtr_registrator_RestartNode;

}

} // Namespace protocol
} // Namespace uavcan

/*
 * YAML streamer specialization
 */
namespace uavcan
{

template <>
class UAVCAN_EXPORT YamlStreamer< ::uavcan::protocol::RestartNode::Request >
{
public:
    template <typename Stream>
    static void stream(Stream& s, ::uavcan::protocol::RestartNode::Request::ParameterType obj, const int level);
};

template <typename Stream>
void YamlStreamer< ::uavcan::protocol::RestartNode::Request >::stream(Stream& s, ::uavcan::protocol::RestartNode::Request::ParameterType obj, const int level)
{
    (void)s;
    (void)obj;
    (void)level;
    if (level > 0)
    {
        s << '\n';
        for (int pos = 0; pos < level; pos++)
        {
            s << "  ";
        }
    }
    s << "magic_number: ";
    YamlStreamer< ::uavcan::protocol::RestartNode::Request::FieldTypes::magic_number >::stream(s, obj.magic_number, level + 1);
}

template <>
class UAVCAN_EXPORT YamlStreamer< ::uavcan::protocol::RestartNode::Response >
{
public:
    template <typename Stream>
    static void stream(Stream& s, ::uavcan::protocol::RestartNode::Response::ParameterType obj, const int level);
};

template <typename Stream>
void YamlStreamer< ::uavcan::protocol::RestartNode::Response >::stream(Stream& s, ::uavcan::protocol::RestartNode::Response::ParameterType obj, const int level)
{
    (void)s;
    (void)obj;
    (void)level;
    if (level > 0)
    {
        s << '\n';
        for (int pos = 0; pos < level; pos++)
        {
            s << "  ";
        }
    }
    s << "ok: ";
    YamlStreamer< ::uavcan::protocol::RestartNode::Response::FieldTypes::ok >::stream(s, obj.ok, level + 1);
}

}

namespace uavcan
{
namespace protocol
{

template <typename Stream>
inline Stream& operator<<(Stream& s, ::uavcan::protocol::RestartNode::Request::ParameterType obj)
{
    ::uavcan::YamlStreamer< ::uavcan::protocol::RestartNode::Request >::stream(s, obj, 0);
    return s;
}

template <typename Stream>
inline Stream& operator<<(Stream& s, ::uavcan::protocol::RestartNode::Response::ParameterType obj)
{
    ::uavcan::YamlStreamer< ::uavcan::protocol::RestartNode::Response >::stream(s, obj, 0);
    return s;
}

} // Namespace protocol
} // Namespace uavcan

#endif // UAVCAN_PROTOCOL_RESTARTNODE_HPP_INCLUDED