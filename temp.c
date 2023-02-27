#include <stdio.h>															// Acesso as opercoes de entradas e/ou saidas.
#include <driver/adc.h>															// Modulo conversor ADC.
#include "freertos/FreeRTOS.h"													// Acesso aos termos.
#include "freertos/task.h"														// Acesso as prioridades da TASK.
#include "driver/gpio.h"														// Acesso ao uso das GPIOs.
#include "esp_log.h"															// Acesso ao uso dos LOGs.
#include <esp_http_server.h>    												// => httpd_uri_t
#include "rom/ets_sys.h"														// Acesso a escala de tempo em micro segundos.
#include <esp_event.h>          												// => esp_event_base_t
#include <nvs_flash.h>          												// => nvs_flash_init
#include <sys/param.h>          												// => MIN()
#include "esp_tls_crypto.h"														// => esp_crypto_base64_encode
#include "esp_netif.h"                                                          // 
#include "esp_wifi.h"                                                           // 

/* RTOS */
#define CONFIG_FREERTOS_HZ 100													// Definicao da Espressif. Escala de tempo base (vTaskDelay).

#define CONFIG_EXAMPLE_GPIO_RANGE_MIN				0
#define CONFIG_EXAMPLE_GPIO_RANGE_MAX				33
#define CONFIG_EXAMPLE_CONNECT_WIFI												// Sim
#define CONFIG_EXAMPLE_WIFI_SSID					"IoT"
#define CONFIG_EXAMPLE_WIFI_PASSWORD				"12345678"
#define CONFIG_EXAMPLE_WIFI_SCAN_METHOD_ALL_CHANNEL	
#define CONFIG_EXAMPLE_WIFI_CONNECT_AP_BY_SIGNAL								// Sim
#define CONFIG_EXAMPLE_CONNECT_IPV6												// Sim
#define CONFIG_EXAMPLE_CONNECT_IPV6_PREF_LOCAL_LINK								// Sim
#define CONFIG_EXAMPLE_BASIC_AUTH												// Sim
#define CONFIG_EXAMPLE_BASIC_AUTH_USERNAME			"ESP32"						// Formulario de acesso: Usuario
#define CONFIG_EXAMPLE_BASIC_AUTH_PASSWORD			"ESPWebServer"				// Formulario de acesso: Senha
#define EXAMPLE_WIFI_SCAN_METHOD                    WIFI_ALL_CHANNEL_SCAN       // 
// #define EXAMPLE_WIFI_SCAN_METHOD                    WIFI_FAST_SCAN
#define EXAMPLE_DO_CONNECT                                                      // 
// Limites de busca
#define CONFIG_EXAMPLE_WIFI_SCAN_RSSI_THRESHOLD		-127
// #define CONFIG_EXAMPLE_WIFI_AUTH_OPEN											// 
// #define CONFIG_EXAMPLE_WIFI_AUTH_WEP
// #define CONFIG_EXAMPLE_WIFI_AUTH_WPA_PSK
#define CONFIG_EXAMPLE_WIFI_AUTH_WPA2_PSK
// #define CONFIG_EXAMPLE_WIFI_AUTH_WPA_WPA2_PSK
// #define CONFIG_EXAMPLE_WIFI_AUTH_WPA2_ENTERPRISE
// #define CONFIG_EXAMPLE_WIFI_AUTH_WPA3_PSK
// #define CONFIG_EXAMPLE_WIFI_AUTH_WPA2_WPA3_PSK
// #define CONFIG_EXAMPLE_WIFI_AUTH_WAPI_PSK
#define EXAMPLE_CONNECT_PREFERRED_IPV6_TYPE         ESP_IP6_ADDR_IS_LINK_LOCAL
// #define EXAMPLE_CONNECT_PREFERRED_IPV6_TYPE         ESP_IP6_ADDR_IS_GLOBAL
// #define EXAMPLE_CONNECT_PREFERRED_IPV6_TYPE         ESP_IP6_ADDR_IS_SITE_LOCAL
// #define EXAMPLE_CONNECT_PREFERRED_IPV6_TYPE         ESP_IP6_ADDR_IS_UNIQUE_LOCAL
#define EXAMPLE_WIFI_CONNECT_AP_SORT_METHOD         WIFI_CONNECT_AP_BY_SIGNAL
// #define EXAMPLE_WIFI_CONNECT_AP_SORT_METHOD         WIFI_CONNECT_AP_BY_SECURITY
// #define EXAMPLE_WIFI_SCAN_AUTH_MODE_THRESHOLD       WIFI_AUTH_OPEN
// #define EXAMPLE_WIFI_SCAN_AUTH_MODE_THRESHOLD       WIFI_AUTH_WEP
// #define EXAMPLE_WIFI_SCAN_AUTH_MODE_THRESHOLD       WIFI_AUTH_WPA_PSK
#define EXAMPLE_WIFI_SCAN_AUTH_MODE_THRESHOLD       WIFI_AUTH_WPA2_PSK
// #define EXAMPLE_WIFI_SCAN_AUTH_MODE_THRESHOLD       WIFI_AUTH_WPA_WPA2_PSK
// #define EXAMPLE_WIFI_SCAN_AUTH_MODE_THRESHOLD       WIFI_AUTH_WPA2_ENTERPRISE
// #define EXAMPLE_WIFI_SCAN_AUTH_MODE_THRESHOLD       WIFI_AUTH_WPA3_PSK
// #define EXAMPLE_WIFI_SCAN_AUTH_MODE_THRESHOLD       WIFI_AUTH_WPA2_WPA3_PSK
// #define EXAMPLE_WIFI_SCAN_AUTH_MODE_THRESHOLD       WIFI_AUTH_WAPI_PSK

// Pelo SDKCONFIG
#define CONFIG_ESP32_WIFI_STATIC_RX_BUFFER_NUM       10
#define CONFIG_ESP32_WIFI_DYNAMIC_RX_BUFFER_NUM      32
#define CONFIG_ESP32_WIFI_TX_BUFFER_TYPE             1
#define CONFIG_ESP32_WIFI_DYNAMIC_TX_BUFFER_NUM      32
#define CONFIG_ESP32_WIFI_TX_BA_WIN                  6
#define CONFIG_ESP32_WIFI_RX_BA_WIN                  6            
#define CONFIG_ESP32_WIFI_SOFTAP_BEACON_MAX_LEN      752
#define CONFIG_ESP32_WIFI_MGMT_SBUF_NUM              32

#define MAX_HTTP_RECV_BUFFER 						512
#define MAX_HTTP_OUTPUT_BUFFER 						2048


/* DHT 11/22 */
#define ___dhtPin	23															// Seleciona o pino de acesso ao DHT 11/22.

/* WiFi */
static const char *TAG = "Wifi_Temperatura";                                           // Identificacao da 'Task'.
int vlrQuery1, vlrQuery2, vlrQuery3;                                            // Var. para 'Query' do metodo 'GET'.
char vlrAscIn[]={"0000"};                                                       // Var. da entrada para retorno HTML.



void int2Asc(unsigned int valor, char *buffer, char digi) 						// Converte INT em ASCII: Valor(Bin), Matriz, Numero de digitos (0 a 5). 
{
	if(digi>5) digi=5;															// Previne erros.
	switch(digi)																// Seleciona o numero de algarismos.
	{
			case 0: 															// Nao converte o Valor(Bin).
				break;															// Retorno.
			case 1:																// Um algarismo.
				buffer[0]=(valor%10)       +0x30; 								// Separa a unidade.
				break;															// Retorno.
			case 2:																// Dois algarismos.
				buffer[0]=(valor/10)       +0x30; 								// Separa a dezena.
				buffer[1]=(valor%10)       +0x30; 								// Separa a unidade.
				break;															// Retorno.
			case 3:																// Tres algarismos.
				buffer[0]=(valor/100)      +0x30; 								// Separa a centena.
				buffer[1]=((valor/10)%10)  +0x30; 								// Separa a dezena.
				buffer[2]=(valor%10)       +0x30; 								// Separa a unidade.
				break;															// Retorno.	
			case 4:																// Quatro algarismos.
				buffer[0]=(valor/1000)     +0x30; 								// Separa a unidade de milhar.
				buffer[1]=((valor/100)%10) +0x30; 								// Separa a centena.
				buffer[2]=((valor/10)%10)  +0x30; 								// Separa a dezena.
				buffer[3]=(valor%10)       +0x30; 								// Separa a unidade.
				break;															// Retorno.
			case 5:																// Cinco algarismos.
				buffer[0]=(valor/10000)    +0x30; 								// Separa a dezena de milhar.
				buffer[1]=((valor/1000)%10)+0x30; 								// Separa a unidade de milhar
				buffer[2]=((valor/100)%10) +0x30; 								// Separa a centena.
				buffer[3]=((valor/10)%10)  +0x30; 								// Separa a dezena.
				buffer[4]=(valor%10)       +0x30; 								// Separa a unidade.
				break;															// Retorno.
	}
}

void adcIniciar(void)															// Inicializa o hardware do modulo conversor ADC.
{
	adc1_config_width(ADC_WIDTH_BIT_12); 										// Config. a resolucao do ADC para 12bits.
	adc1_config_channel_atten(ADC1_CHANNEL_0,ADC_ATTEN_DB_11); 					// Config. a atenuacao do Canal 0 (GPIO36).
	adc1_config_channel_atten(ADC1_CHANNEL_3,ADC_ATTEN_DB_11); 					// Config. a atenuacao do Canal 3 (GPIO39).
	/*
	Attenuation			Measurable input voltage range
	ADC_ATTEN_DB_0		100 mV ~ 950 mV
	ADC_ATTEN_DB_2_5	100 mV ~ 1250 mV
	ADC_ATTEN_DB_6		150 mV ~ 1750 mV
	ADC_ATTEN_DB_11 	150 mV ~ 2450 mV
	*/
}

/* Bloco Inicio: DHT 11/22 */
unsigned char   timeOut=0;														// Flag que indica o limite do tempo.
unsigned int    valor16bRh, valor16bTp;											// Var. global para salvar os valores do DHT 11/22.

struct dnthRegistro_t															// Estrutura de armazenamento.
{
    unsigned char rhInteiro;													// Parte inteira do valor da umidade.
    unsigned char rhDecimal;													// Parte decimal do valor da umidade.
    unsigned char tempInteiro;													// Parte inteira do valor da temperatura.
    unsigned char tempDecimal;													// Parte decimal do valor da temperatura.
    unsigned char checksum;
}dnthRegistro;																	// Variavel de acesso aos dados.

void dhtxxIniciar(void)															// Deve ser utilizada para disparar a leitura.
{
	gpio_reset_pin(___dhtPin);													// Limpa configuracoes anteriores.
	gpio_set_direction(___dhtPin, GPIO_MODE_OUTPUT);							// Configura o pino como saida.
	gpio_set_level(___dhtPin,0);                                             	// Coloca o Pino em '0'.
	ets_delay_us(20 * 1000);									                // Aguardar 20ms.
	gpio_set_level(___dhtPin,1);                                             	// Coloca o Pino em '1'.
	ets_delay_us(30);									                		// Aguardar 30us.
	gpio_set_direction(___dhtPin, GPIO_MODE_INPUT);								// Configura o pino como entrada.
}

char dhtxxLer(void)																// Executa a leitura dos bits com limite de tempo.
{
	char i, dado = 0;															// Var. temporaria.
	unsigned int k;        														// 'k' eh usado para contar 1 bit durante a leitura.
	if(timeOut) return (0);														// Se o tempo jah estorou o limite... volta.
	for(i = 0; i < 8; i++)														// Laco de leitura com limite de tempo.
	{
		k = 0;																	// Inicia o processo.
		while(!(gpio_get_level(___dhtPin) & 1))                 				// Laco que aguarda (subida do sinal) com timeout.
		{ 
			k++;																// Incrementa.
			if(k > 10000)														// Verifica se esta no limite do tempo.
			{
				timeOut = 1;													// Estourou o limite do tempo, ativa a flag.
				break;															// Interrompe o processo.
			}
			ets_delay_us(1);													// Aguarda 1us. Limite da largura do sinal.
		}
		ets_delay_us(30);														// Aguarda 1us. Limite da mudanca de nivel do sinal.
		if(gpio_get_level(___dhtPin)==1) dado=((dado<<1) | 1);					// Verifica se o dado eh '0' ou '1'. Se ativado, agregar o valor.
		else dado=(dado<<1);                    								// Senao, apenas desloca um bit a esquerda.
      
		while(gpio_get_level(___dhtPin) ==1)                 					// Laco que aguarda (descida do sinal) com timeout.
		{ 
            k++;																// Incrementa.
            if(k > 10000)														// Verifica se esta no limite do tempo.
            {
                timeOut = 1;													// Estourou o limite do tempo, ativa a flag.
                break;															// Interrompe o processo.
            }
			ets_delay_us(1);													// Aguarda 1us. Limite da largura do sinal.
		}
    }
  return (dado);																// Retorna com valor lido.
}

char dhtxxChecar(void)															// Verifica a resposta do DHT 11/22.
{
	ets_delay_us(40);															// Aguarda 40us. Limite do tempo de resposta anterior.
	if(gpio_get_level(___dhtPin)==0)											// Testa o pino se esta em 0.
	{
		ets_delay_us(80);														// Aguarda 80us. Limite do tempo de resposta.
		if(gpio_get_level(___dhtPin)==1)										// Testa o pino se esta em 1.
    	{
			ets_delay_us(50);													// Aguarda 50us. Limite do tempo de resposta.
			return (1);															// Ok!
    	}
	}
	return (0);																	// Falhou!
}

char __dhtHex2Dec(char valor) 													// Converte os valores do DHT11.
{
    unsigned cont, tst=1, charconv=0;											// Var. local temporaria.
    for(cont=0;cont<8;cont++) 													// Laco de varredura do Byte.
    {
		if((bitX(valor,cont))==1) charconv = charconv + tst;					// Testa todos os bits.
        tst += tst;																// Dobra o valor (1,2,4,8...).
    }
    return (charconv);															// Retorna o valor convertido.
}

char dhtxxValor(char valordht) 													// Faz leitura do do DHT escolhido.
{
    char testar, temp;															// Var. local temporaria.
    if(valordht<1)valordht=1;													// Limita o valor minimo.
    if(valordht>2)valordht=2;													// Limita o valor maximo.
    if(dhtxxChecar())															// Verifica a comunicacao antes de ler.
    {
        dnthRegistro.rhInteiro=dhtxxLer();										// Faz a leitura do 1o. grupo de dados e salva.
        dnthRegistro.rhDecimal=dhtxxLer();										// Faz a leitura do 2o. grupo de dados e salva.
        dnthRegistro.tempInteiro=dhtxxLer();									// Faz a leitura do 3o. grupo de dados e salva.
        dnthRegistro.tempDecimal=dhtxxLer();									// Faz a leitura do 4o. grupo de dados e salva.
        dnthRegistro.checksum=dhtxxLer();										// Faz a leitura do 5o. grupo de dados e salva.
    }
    
    if(valordht==1)																// Se for o DHT11...
    {
        temp = __dhtHex2Dec(dnthRegistro.rhInteiro);							// Converte o valor da umidade.
        valor16bRh = temp * 10;													// Corrige o valor.
        temp = __dhtHex2Dec(dnthRegistro.tempInteiro);							// Converte o valor da temperatura.
        valor16bTp = temp * 10;													// Corrige o valor.
    }
    
    if(valordht==2)																// Se for o DHT22...
    {
        valor16bRh = (dnthRegistro.rhInteiro<<8) | (dnthRegistro.rhDecimal);	// Combina parte inteira e decimal da umidade.
        valor16bTp = (dnthRegistro.tempInteiro<<8) | (dnthRegistro.tempDecimal);// Combina parte inteira e decimal da temperatura.
    }
    
	// Verifica se o conteudo dos dados sao validos.
    testar=dnthRegistro.rhInteiro+dnthRegistro.rhDecimal+dnthRegistro.tempInteiro+dnthRegistro.tempDecimal;
    if (dnthRegistro.checksum !=testar) return (0);								// Falha de leitura ou dados invalidos.
    else return (1);															// Ok!
}

void dhtxx(unsigned char modelo,unsigned int *umidade,unsigned int *temperatura)// Acesso ao DHT por modelo e variaveis.
{	
	if(dhtxxValor(modelo))														// Verifica se os dados sao validos...
	{
		*umidade = valor16bRh;													// Salva o valor da umidade.
		*temperatura = valor16bTp;												// Salva o valor da temperatura.
	}
}

static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    if (event_id == WIFI_EVENT_AP_STACONNECTED) 
    {
        wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*) event_data;
        ESP_LOGI(TAG, "Estacao "MACSTR" entrou, AID=%d", MAC2STR(event->mac), event->aid);
    } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) 
    {
        wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*) event_data;
        ESP_LOGI(TAG, "Estacao "MACSTR" saiu, AID=%d", MAC2STR(event->mac), event->aid);
    }
}

void wifi_init_softap(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = CONFIG_ESP_WIFI_SSID,
            .ssid_len = strlen(CONFIG_ESP_WIFI_SSID),
            .channel = CONFIG_ESP_WIFI_CHANNEL,
            .password = CONFIG_ESP_WIFI_PASSWORD,
            .max_connection = CONFIG_ESP_MAX_STA_CONN,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK,
            // .pmf_cfg = {
            //         .required = 0,
            // },
        },
    };

    // wifi_pmf_config_t wifi_pmf = {
    //     {.required = 0}
    // };

    if (strlen(CONFIG_ESP_WIFI_PASSWORD) == 0) 
    {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

	esp_wifi_connect();
    ESP_LOGI(TAG, "Soft AP Ok: SSID:%s Pass:%s Canal:%d", CONFIG_ESP_WIFI_SSID, CONFIG_ESP_WIFI_PASSWORD, CONFIG_ESP_WIFI_CHANNEL);
}

void nvs_init(void)
{
    //Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) 
    {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "Modo: AP");
}

esp_err_t get_handler(httpd_req_t *req)
{
    const char resp[] = "<!DOCTYPE html><html><head><title>ESP-IDF Fita Led</title>\
<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\"></head>\
<body><style>*{font-family: Verdana;text-align: center;}</style>\
<h2>Color Picker</h2><form action=\"\"><label for=\"rgb\"><h3>Cor:</h3></label>\
<input type=\"color\" id=\"rgb\" name=\"rgb\" value=\"#0000ff\"><br><br><input type=\"submit\">\
</form></body></html>";

    httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

esp_err_t get_handler_rgb(httpd_req_t *req)
{
    /**/
    const char resp[] = "<!DOCTYPE html><html><head><title>ESP-IDF Fita Led</title>\
<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\"></head>\
<body><style>*{font-family: Verdana;text-align: center;}</style>\
<h2>Color Picker</h2><form action=\"\" method=\"get\"><label for=\"rgb\"><h3>Cor:</h3></label>\
<input type=\"color\" id=\"rgb\" name=\"rgb\" value=\"#0000ff\"><br><br><input type=\"submit\">\
</form></body></html>";

    httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
    /**/

    // Le a linha da URI e pega o host.
    char *buf;
    size_t buf_len;
    buf_len = httpd_req_get_hdr_value_len(req, "Host") + 1;
    if (buf_len > 1)
    {
        buf = malloc(buf_len);
        if (httpd_req_get_hdr_value_str(req, "Host", buf, buf_len) == ESP_OK)
        {
            ESP_LOGI(TAG, "Host: %s", buf);
        }
        free(buf);
    }

    // Le a linha da URI e pega o(s) parametro(s).
    buf_len = httpd_req_get_url_query_len(req) + 1;
    if (buf_len > 1) {
        buf = malloc(buf_len);
        if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK) {
            ESP_LOGI(TAG, "Dado na URL: %s", buf);
            char param[32]; // era 32
            if (httpd_query_key_value(buf, "temp", param, sizeof(param)) == ESP_OK) 
            {
                ESP_LOGI(TAG, "Valor temperatura= %s", param);
                hex2dec(param);
                //printf("Valor R= %d\n", valorR);
                //printf("Valor G= %d\n", valorG);
                //printf("Valor B= %d\n", valorB);
            }
        }
        free(buf);
    }
    get_handler(req);
    return ESP_OK;
}

/* Manipulador da estrutura da URI para metodo GET /uri */
httpd_uri_t uri_get = 
    {
    .uri = "/",
    .method = HTTP_GET,
    .handler = get_handler,
    .user_ctx = NULL
    };

httpd_uri_t uri_get_rgb = 
    {
    .uri = "/temperatura",
    .method = HTTP_GET,
    .handler = get_handler_rgb,
    .user_ctx = NULL
    };

httpd_handle_t start_webserver(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    httpd_handle_t server = NULL;
    if (httpd_start(&server, &config) == ESP_OK)
    {
        httpd_register_uri_handler(server, &uri_get);
        // httpd_register_uri_handler(server, &uri_get_saidas);
        // httpd_register_uri_handler(server, &uri_get_entradas);
        httpd_register_uri_handler(server, &uri_get_rgb);
    }
    return server;
}

void stop_webserver(httpd_handle_t server)
{
    if (server)
    {
        httpd_stop(server);
    }
}

void wifi_init( void )
{
    nvs_init();
    wifi_init_softap();
    // esp32Ap();
    // server_initiation();
    start_webserver();
}


void app_main(void)
{
    if(vlrTecla=='A')														// Se a tecla for acionada...
		{
			vlrAn0 = adc1_get_raw(ADC1_CHANNEL_0);								// Le o pino GPIO36.
			vlrAn1 = adc1_get_raw(ADC1_CHANNEL_3);								// Le o pino GPIO39.
			int2Asc(vlrAn0,vlrAsc0,4); 											// Converte o valor 'int' em ASCII.
			int2Asc(vlrAn1,vlrAsc1,4); 											// Converte o valor 'int' em ASCII.
			lcdString(vlrAsc1,1,1); 											// Envia o valor 'X' convertido ao LCD.
			lcdString(vlrAsc0,1,7); 											// Envia o valor 'Y' convertido ao LCD.
			vTaskDelay(100); 									                // Aguarda 1000ms.
		}


        	/* Teste do DHT11 */
		if(vlrTecla=='C')														// Se a tecla for acionada...
		{
			dhtxxIniciar();														// Dispara a leitura no DHT 11.
			dhtxx(1,&umidade,&temperatura);										// Processo de selecao e armazenamento.
			int2Asc(umidade,vlrAsc2,4); 										// Converte o valor 'int' em ASCII.
			int2Asc(temperatura,vlrAsc3,4); 									// Converte o valor 'int' em ASCII.
			lcdString(vlrAsc2,2,1);												// Envia ao LCD.
			lcdString(vlrAsc3,2,7);												// Envia ao LCD.
			vTaskDelay(100); 									                // Aguarda 1000ms.
		}

        /* Teste do DHT22 */
		if(vlrTecla=='D')														// Se a tecla for acionada...
		{
			dhtxxIniciar();														// Dispara a leitura no DHT 22.
			dhtxx(2,&umidade,&temperatura);										// Processo de selecao e armazenamento.
			int2Asc(umidade,vlrAsc2,4); 										// Converte o valor 'int' em ASCII.
			int2Asc(temperatura,vlrAsc3,4); 									// Converte o valor 'int' em ASCII.
			lcdString(vlrAsc2,2,1);												// Envia ao LCD.
			lcdString(vlrAsc3,2,7);												// Envia ao LCD.
			vTaskDelay(100); 									                // Aguarda 1000ms.
		}
		/* */


    while (1)
    {
    /* code */


    }
    vTaskDelay(10);

}
