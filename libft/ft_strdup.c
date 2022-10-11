/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   ft_strdup.c                                        :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: youskim <youskim@student.42seoul.kr>       +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2021/11/17 15:42:59 by youskim           #+#    #+#             */
/*   Updated: 2021/11/21 17:30:14 by youskim          ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include <stdlib.h>

char	*ft_strdup(const char *str)
{
	int		len;
	char	*arr;

	len = 0;
	while (str[len])
		len++;
	arr = (char *)malloc(sizeof(char) * (len + 1));
	if (arr == 0)
		return (NULL);
	len = 0;
	while (str[len])
	{
		arr[len] = str[len];
		len++;
	}
	arr[len] = '\0';
	return (arr);
}
